// fpga가 가속도 센서에게 주소 송신 요청 후 데이터 받아와서 fpga로 전달하는 모듈 (실제 통신 모듈)
module spi_controller (		
							iRSTN,
							iSPI_CLK,
							iSPI_CLK_OUT,
							iP2S_DATA,
							iSPI_GO,
							oSPI_END,					
							oS2P_DATA,							
							SPI_SDIO,
							oSPI_CSN,							
							oSPI_CLK);
	
`include "spi_param.h"	//헤더 파일

//=======================================================
//  PORT declarations
//=======================================================
//	Host Side
input				              iRSTN;
input				              iSPI_CLK;
// 위상차 클럭
input				              iSPI_CLK_OUT;
// 병렬로 받은 데이터 직렬로 출력
input	      [SI_DataL:0]  iP2S_DATA; 
// 시작 신호 (1이되면 통신 시작)
input	      			        iSPI_GO;
// 종료 신호
output	  			          oSPI_END;
// 직렬로 받은 데이터 병렬로 출력
output	reg [SO_DataL:0]	oS2P_DATA;
//	bidirectional SPI  
inout				              SPI_SDIO;
// 칩 선택 신호 active low
output	   			          oSPI_CSN;
// spi 클럭
output				            oSPI_CLK;

//=======================================================
//  REG/WIRE declarations
//=======================================================
wire          read_mode, write_address;

reg           spi_count_en;
// 16비트 데이터 처리하기 때문에 4비트 카운터
reg  	[3:0]		spi_count;

//=======================================================
//  Structural coding
//=======================================================

// 보낼 데이터가 1이면 읽기 모드, 0이면 쓰기 모드
assign read_mode = iP2S_DATA[SI_DataL];
// 15에서 0까지 감소하는 카운터이므로 MSB가 1이면 주소 쓰기
assign write_address = spi_count[3];
// 카운터가 0이되면 종료됐음을 알림
assign oSPI_END = ~|spi_count;
// 시작 신호가 1이면 CS 신호 활성화
assign oSPI_CSN = ~iSPI_GO;
// SPI 클럭은 카운터가 동작 중일 때만 출력, 아니면 High 상태 유지
assign oSPI_CLK = spi_count_en ? iSPI_CLK_OUT : 1'b1;
// SPI 데이터 라인은 쓰기 모드이거나 주소 쓰기 모드일 때만 입력, 아니면 High-Z (tri-state bus)
assign SPI_SDIO = spi_count_en && (!read_mode || write_address) ? iP2S_DATA[spi_count] : 1'bz;

always @ (posedge iSPI_CLK or negedge iRSTN) 
	if (!iRSTN)
	// 초기화
	begin
		spi_count_en <= 1'b0;
		spi_count <= 4'hf;
	end
	else 
	begin
		// 끝나면 카운터 작동 중지
		if (oSPI_END)
			spi_count_en <= 1'b0;
			// 동작 신호에 의해 카운터 동작 시작
		else if (iSPI_GO)
			spi_count_en <= 1'b1;
		// 카운터 동작하지 않으면 초기화	
		if (!spi_count_en)	
  		spi_count <= 4'hf;
		// 카운터 동작 		
		else
			spi_count	<= spi_count - 4'b1;
	// 읽기 모드이고 주소 전송이 끝났을 때
	// 1비트씩 데이터 쉬프트하고 입력받은 데이터로 채움 
    if (read_mode && !write_address)
		  oS2P_DATA <= {oS2P_DATA[SO_DataL-1:0], SPI_SDIO};
	end

endmodule