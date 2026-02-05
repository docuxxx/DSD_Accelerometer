/* 본 모듈은 ADXL345 가속도 센서를 SPI 인터페이스로 제어하며, 
초기 설정 단계에서 activity/inactivity, free-fall, 데이터 레이트 및 인터럽트 매핑을 구성한다. 
초기화 이후에는 INT2 인터럽트를 기반으로 상태 레지스터를 확인하고, 데이터 준비 시 X축 가속도 데이터를 16비트로 분할 읽기하여 출력한다. 
Activity는 DC-coupled 방식으로 절대 가속도를 기준으로 판단하고, 
Inactivity는 AC-coupled 방식으로 기준 대비 변화량을 비교함으로써 중력 및 설치 자세 변화 동작 검출을 수행한다. */
module spi_ee_config (			
								iRSTN,															
								iSPI_CLK,								
								iSPI_CLK_OUT,								
								iG_INT2,
								oDATA_L,
								oDATA_H,
								SPI_SDIO,
								oSPI_CSN,
								oSPI_CLK);

			
`include "spi_param.h"
	
//=======================================================
//  PORT declarations
//=======================================================
//	Host Side							
input					          iRSTN;
input					          iSPI_CLK, iSPI_CLK_OUT;
input					          iG_INT2;
output reg [SO_DataL:0] oDATA_L;
output reg [SO_DataL:0] oDATA_H;
//	SPI Side           
inout					          SPI_SDIO;
output					        oSPI_CSN;
output					        oSPI_CLK;       
                               
//=======================================================
//  REG/WIRE declarations
//=======================================================
reg	    [3:0] 	       ini_index;
// 어느 주소에 데이터를 쓸지 
reg		  [SI_DataL-2:0] write_data;
// 16비트 전체 데이터
reg		  [SI_DataL:0]	 p2s_data;
reg                    spi_go;
wire                   spi_end;
wire	  [SO_DataL:0]	 s2p_data; // SPI로부터 받아온 데이터 INT_SOURCE 
reg     [SO_DataL:0]	 low_byte_data;
reg		       		       spi_state;
reg                    high_byte; // indicate to read the high or low byte
reg                    read_back; // indicate to read back data (1이면 data, 0이면 status) 
reg                    clear_status, read_ready;
reg     [3:0]          clear_status_d;
reg                    high_byte_d, read_back_d;
reg	    [IDLE_MSB:0]   read_idle_count; // reducing the reading rate

//=======================================================
//  Sub-module
//=======================================================
spi_controller u_spi_controller (		
							.iRSTN(iRSTN),
							.iSPI_CLK(iSPI_CLK),
							.iSPI_CLK_OUT(iSPI_CLK_OUT),
							.iP2S_DATA(p2s_data),
							.iSPI_GO(spi_go),
							.oSPI_END(spi_end),			
							.oS2P_DATA(s2p_data),			
							.SPI_SDIO(SPI_SDIO),
							.oSPI_CSN(oSPI_CSN),							
							.oSPI_CLK(oSPI_CLK));
							
//=======================================================
//  Structural coding
//=======================================================
// Initial Setting Table
always @ (ini_index)
	case (ini_index)
	// 가속도 센서 초기 설정값 LUT 구조
	// 가속도 센서 데이터시트 22page 참조
    0      : write_data = {THRESH_ACT,8'h20}; // 2g 감지시 동작 감지
    1      : write_data = {THRESH_INACT,8'h03}; // 0.18g 감지시 비동작 감지
    2      : write_data = {TIME_INACT,8'h01}; // 0.18g 이하 1초간 비동작시 비동작 상태로 간주
    3      : write_data = {ACT_INACT_CTL,8'h7f}; // 모든 축에 대해 동작/비동작 감지 0111_1111 DC로 움직임 감지 AC로 멈춤 감지 
    4      : write_data = {THRESH_FF,8'h09}; // 0.56g 보다 작아지면 낙하
    5      : write_data = {TIME_FF,8'h46}; // 0.56g 이하로 숫자 70 (70 x 5ms = 0.35s) 동안 낙하 상태 유지 시 낙하로 판단. (최대치)
    6      : write_data = {BW_RATE,8'h09}; // output data rate : 50 Hz, normal power mode (4번째 비트 0)
    7      : write_data = {INT_ENABLE,8'h10};	// 움직임이 있을 때만 인터럽트 신호 1 출력
    8      : write_data = {INT_MAP,8'h10}; // activity 인터럽트만 INT2 핀에 매핑, 나머진 INT1 핀에 매핑되어 OR'ed로 출력
	// 3-wire SPI(D6), Active High(D5), 10bit resolution(D3), 오른쪽 정렬(D2), +/-2g (D1,D0)
    9      : write_data = {DATA_FORMAT,8'h40}; 
	// Link (D5), Auto Sleep(D4), Wakeup(D1,D0) 사용 x, 항상 측정 중 
	  default: write_data = {POWER_CONTROL,8'h08};
	endcase

always@(posedge iSPI_CLK or negedge iRSTN)
	if(!iRSTN)
	// 리셋
	begin
		ini_index	<= 4'b0;
		spi_go		<= 1'b0;
		spi_state	<= IDLE;
		// 정기적 status 체크, 즉, 인터럽트가 발생했는지 확인 후 새로운 가속도 데이터가 준비됐는지 확인 
		read_idle_count <= 0; // read mode only
		high_byte <= 1'b0; // read mode only
		read_back <= 1'b0; // read mode only
    clear_status <= 1'b0;
	end
	// initial setting (write mode)
	else if(ini_index < INI_NUMBER) 
		case(spi_state)
			IDLE : begin
				    // 주소와 데이터 전송
					p2s_data  <= {WRITE_MODE, write_data};
					spi_go		<= 1'b1;
					spi_state	<= TRANSFER;
			end
			TRANSFER : begin
					// 전송 완료시 
					if (spi_end)
					begin
						// 다음 주소로 이동 및 초기 상태 
		        ini_index	<= ini_index + 4'b1;
						spi_go		<= 1'b0;
						spi_state	<= IDLE;							
					end
			end
		endcase
  // read data and clear interrupt (read mode)

  else 
		case(spi_state)
		// SPI 인터페이스는 8비트 직렬 통신이므로 가속도 센서의 데이터 16비트를 8비트씩 나눠서 읽어야함 
			IDLE : begin
				  read_idle_count <= read_idle_count + 1;
					// 상위 바이트 읽을 차례 (상위 바이트 먼저 읽음)
					if (high_byte) // multiple-byte read
				  begin
						// 읽기 모드로 X축 상위 바이트 읽기
					  p2s_data[15:8] <= {READ_MODE, X_HB};						
					  read_back      <= 1'b1;
					end
				  // 하위 바이트 읽을 차례
				  else if (read_ready)
				  begin
					  // 읽기 모드로 X축 하위 바이트부터 읽기
					  p2s_data[15:8] <= {READ_MODE, X_LB};						
					  read_back      <= 1'b1;
					end
				  // 상태 레지스터 읽기 (상태 확인)
				  else if (!clear_status_d[3]&&iG_INT2 || read_idle_count[IDLE_MSB])
				  begin
					  p2s_data[15:8] <= {READ_MODE, INT_SOURCE};
					  clear_status   <= 1'b1;
          end
		// SPI 통신 시작 
          if (high_byte || read_ready || read_idle_count[IDLE_MSB] || !clear_status_d[3]&&iG_INT2)
          begin
					  spi_go		<= 1'b1;
					  spi_state	<= TRANSFER;
					end
				  // 받아온 데이터 저장 
				  if (read_back_d) // update the read back data
				  begin
				  	if (high_byte_d) // 상위 바이트 읽었다면 하위 바이트와 합쳐서 저장
				  	begin
				  	  oDATA_H <= s2p_data;	
				  	  oDATA_L <= low_byte_data;			  		
				  	end
				  	else // 하위 바이트 읽었다면 저장 
				  		low_byte_data <= s2p_data;
				  end
			end
			TRANSFER : begin
					// 전송 완료시 
					if (spi_end)
					begin
						spi_go		<= 1'b0;
						spi_state	<= IDLE;
						// 읽고 나서 
						if (read_back)
						begin
							read_back <= 1'b0;
					    high_byte <= !high_byte;
					    read_ready <= 1'b0;					
					  end
					  else
					  begin
              clear_status <= 1'b0;
              read_ready <= s2p_data[6];	// 1비트 밀려서 6번째 인덱스인 Data Ready 비트 확인 				  	
					    read_idle_count <= 0;
            end
					end
			end
		endcase
 
always@(posedge iSPI_CLK or negedge iRSTN)
	if(!iRSTN)
	begin
		high_byte_d <= 1'b0;
		read_back_d <= 1'b0;
		clear_status_d <= 4'b0;
	end
	else
	begin
		// 한 클럭  뒤 값 
		high_byte_d <= high_byte;
		read_back_d <= read_back;
		// 최소 4클럭 이후에 클리어 신호 반영 (인터럽트 신호 확인 후 삭제가 clear_status 신호)
		clear_status_d <= {clear_status_d[2:0], clear_status};
	end

endmodule					