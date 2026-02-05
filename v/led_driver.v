module led_driver (iRSTN, iCLK, iDIG, iG_INT2, oLED);
input				       iRSTN;
input				       iCLK;
input		    [9:0]  iDIG;
input		           iG_INT2;
output	    [9:0]  oLED;

//=======================================================
//  REG/WIRE declarations
//=======================================================
wire				[4:0]  select_data;
wire               signed_bit;
wire				[3:0]  abs_select_high;
reg				  [1:0]  int2_d;
reg	        [23:0] int2_count;
reg	               int2_count_en;

//=======================================================
//  Structural coding
//=======================================================
// +-2g resolution : 10-bit
assign select_data = iG_INT2 ? iDIG[9:5] :  // 상위 5비트만 사용 - 노이즈 (하위 비트) 버리고 기울기 표현의 필수적인 상위 비트만 남김 
								// +-1g resolution mode에서 부호에 따라  최대/최소 값을 지정하여 LED 표현 범위 내에 출력으로 제한 
                               (iDIG[9]?(iDIG[8]?iDIG[8:4]:5'h10):(iDIG[8]?5'hf:iDIG[8:4])); // +-g resolution : 9-bit       
// select_data의 MSB가 부호 비트                        
assign signed_bit = select_data[4];
// 음수일 때 2의 보수 - 1 값 계산. 즉 절대값 만들기
// LED 인덱스(0부터 시작) 맞추기 위해서 -1 안 함
assign abs_select_high = signed_bit ? ~select_data[3:0] : select_data[3:0]; // the negitive number here is the 2's complement - 1

/*assign oLED = int2_count[23] ? ((abs_select_high[3:1] == 3'h0) ? 8'h18 :
				                        (abs_select_high[3:1] == 3'h1) ? (signed_bit?8'h8:8'h10) :
				                        (abs_select_high[3:1] == 3'h2) ? (signed_bit?8'hc:8'h30) :
				                        (abs_select_high[3:1] == 3'h3) ? (signed_bit?8'h4:8'h20) :
				                        (abs_select_high[3:1] == 3'h4) ? (signed_bit?8'h6:8'h60) :
				                        (abs_select_high[3:1] == 3'h5) ? (signed_bit?8'h2:8'h40) :
				                        (abs_select_high[3:1] == 3'h6) ? (signed_bit?8'h3:8'hc0) :
				                                                         (signed_bit?8'h1:8'h80)):
				                        (int2_count[20] ? 8'h0 : 8'hff); // Activity*/
// 가속도 센서의 절대값에 따라 약 0.16초간 LED 점등												
assign oLED = int2_count[23] ? ((abs_select_high[3:0] == 3'h0) ? 10'h030 : // 수평
				                        (abs_select_high[3:0] == 3'h1) ? (signed_bit?10'h020:10'h010) : // 양/음수 구분해서 1칸 옆이 켜짐
				                        (abs_select_high[3:0] == 3'h2) ? (signed_bit?10'h060:10'h018) : // 양/음수 구분해서 2칸 옆이 켜짐
				                        (abs_select_high[3:0] == 3'h3) ? (signed_bit?10'h040:10'h8) : // 이하 동일
				                        (abs_select_high[3:0] == 3'h4) ? (signed_bit?10'h0C0:10'hC) :
				                        (abs_select_high[3:0] == 3'h5) ? (signed_bit?10'h080:10'h4) :
				                        (abs_select_high[3:0] == 3'h6) ? (signed_bit?10'h180:10'h6) :
				                        (abs_select_high[3:0] == 3'h7) ? (signed_bit?10'h100:10'h2) :
				                        (abs_select_high[3:0] == 3'h8) ? (signed_bit?10'h300:10'h3) :
																		 // 기울기가 8을 넘어가면 맨 끝 LED만 켜짐
				                                                         (signed_bit?10'h200:10'h1)):
										// 약 0.02초마다 LED 전체 Blink (충격 감지) 
				                        (int2_count[20] ? 10'h0 : 10'h3ff); // Activity												
												
												
												
// INT2 인터럽트 신호로부터 상승 에지 검출 및 카운터 시작
always@(posedge iCLK or negedge iRSTN)
	if (!iRSTN)
  begin
    int2_count_en	<= 1'b0;	
    int2_count <= 24'h800000;
  end
	else
	begin
		// 현재 신호와 이전 신호 저장
		int2_d <= {int2_d[0], iG_INT2};
		// 상승 에지 검출 -> 인터럽트 발생
		if (!int2_d[1] && int2_d[0])
	// 인터럽트 발생 시 카운터 시작
    begin
      int2_count_en	<= 1'b1;		
	    int2_count <= 24'h0;
	  end
	  else if (int2_count[23])
	  	int2_count_en	<= 1'b0; 	
    else
	  	int2_count <= int2_count + 1;
	end

endmodule