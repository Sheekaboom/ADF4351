#include "ADF4351.hpp"
    
ADF4351::ADF4351(uint8_t data_pin_in, uint8_t clk_pin_in){
    //first save these valuse to our properties
    data_pin = data_pin_in; clk_pin = clk_pin_in; ss_pin = ADF4351_SS_PIN;
    //now initialize SPI
	SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    SPI.begin();
    delay(500);
    pinMode(ss_pin,OUTPUT);
    digitalWrite(ss_pin,HIGH); //init slave select
    
  }
  
 /* ADF4351::ADF4351(SpiBus spi_bus_in){
    my_spi_bus = spi_bus_in
    
  }*/
  
int8_t ADF4351::init(){//initialize to default 45 MHz
  CHECK_ERR(R[5].set(R5_DEF));
  CHECK_ERR(R[4].set(R4_DEF));
  CHECK_ERR(R[3].set(R3_DEF));
  CHECK_ERR(R[2].set(R2_DEF));
  CHECK_ERR(R[1].set(R1_DEF));
  CHECK_ERR(R[0].set(R0_DEF));
  this->write_all();
	return 0;
  }
	  
  //the full range of the frequency generator was given up for speed
  //a different slower function will be added to allow usage of frequencies from
  //4,294,967,296 Hz to 4,400,000,000
int8_t ADF4351::set_freq(uint32_t freq_hz){

	  //f_pfd = Ref_in * [(1+ref_doub_bit)/(ref_div_bit*(1+div_by_2_bit))] 
	  //calculate f_pfd
	  uint8_t r_dub_val = this->get_reference_doubler();
	  uint8_t r_div_val = this->get_r_counter();
    uint8_t r_div2    = this->get_rdiv2();
	  uint32_t f_pfd_hz = (REF_FREQ_RAW*(1+r_dub_val))/(r_div_val*(1+r_div2));
	  uint32_t freq_khz = freq_hz/1000;
	  //get our division factor jsut gonna use a if else for now (may be able to get faster)
	  uint8_t rf_div = 0;
	  if(freq_khz>MAX_VCO_FREQ_KHZ){return -1;} //error cannot go higher than this
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_1)){rf_div = RF_DIV_1;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_2)){rf_div = RF_DIV_2;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_4)){rf_div = RF_DIV_4;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_8)){rf_div = RF_DIV_8;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_16)){rf_div = RF_DIV_16;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_32)){rf_div = RF_DIV_32;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_64)){rf_div = RF_DIV_64;}
	  else {return -2;} //not in range (below)

	  //now we can get our pfd value divided by our RF divider (still in hz)
	  //RF_out = [INT_div_fact + (FRAC/MOD)]*(f_pfd/RF_div)
	  uint32_t f_pfd_rf_div_hz = f_pfd_hz>>rf_div; //first get our divided down f_pfd
	  //uint32_t f_pfd_rf_div_khz = f_pfd_rf_div_hz/1000; //same thing in khz
	  uint32_t my_int = freq_hz/(f_pfd_rf_div_hz); //calculate our int value
	  uint32_t my_res_hz = freq_hz-(my_int*f_pfd_rf_div_hz); //whats left in khz
	  uint32_t my_mod = this->get_mod(); //get the closest value from the current step value
	  uint32_t my_frac = (uint32_t)((uint64_t)(((uint64_t)my_mod)*((uint64_t)my_res_hz))/((uint64_t)f_pfd_rf_div_hz));
	  
	  //now set the values in the corresponding data structures
   Serial.print("PFD_RF = "); Serial.println(f_pfd_rf_div_hz);
   //Serial.print("PFD_RFK= "); Serial.println(f_pfd_rf_div_khz);
   Serial.print("FREQ_K = "); Serial.println(freq_khz);
   Serial.print("MOD    = "); Serial.println(my_mod);
   Serial.print("RES    = "); Serial.println(my_res_hz);
   Serial.print("RF_DIV = "); Serial.println(rf_div);
   Serial.print(" INT   = "); Serial.println(my_int);
   Serial.print(" FRAC  = "); Serial.println(my_frac);
	  this->set_rf_divider_select(rf_div); //set our rf divider
	  this->set_int((uint16_t)my_int); //set integer value
	  this->set_frac((uint16_t)my_frac); //set frac
	  
	  //now write out all of our registers to the ADF board
   //doing weird stuff with the board so stop for now while debugging
	  this->write_all();
	  return 0;
  }

//should allow us to reach full frequency range, but with some speed (and rounding) drawbacks
//will take the frequency as the sum of the freq_hz and freq_khz
//freq_hz can only be used up to the size of a uint32_t (~4.3GHz) combine both for higher freqs
//NEEDS TESTING
int8_t ADF4351::set_freq_full(uint32_t freq_khz_in,uint32_t freq_hz_in){

	  //f_pfd = Ref_in * [(1+ref_doub_bit)/(ref_div_bit*(1+div_by_2_bit))] 
	  //calculate f_pfd
	  uint8_t r_dub_val = this->get_reference_doubler();
	  uint8_t r_div_val = this->get_r_counter();
      uint8_t r_div2    = this->get_rdiv2();
	  uint32_t f_pfd_hz = (REF_FREQ_RAW*(1+r_dub_val))/(r_div_val*(1+r_div2));
	  uint32_t freq_khz = freq_hz_in/1000+freq_khz_in;
	  //get our division factor jsut gonna use a if else for now (may be able to get faster)
	  uint8_t rf_div = 0;
	  if(freq_khz>MAX_VCO_FREQ_KHZ){return -1;} //error cannot go higher than this
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_1)){rf_div = RF_DIV_1;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_2)){rf_div = RF_DIV_2;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_4)){rf_div = RF_DIV_4;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_8)){rf_div = RF_DIV_8;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_16)){rf_div = RF_DIV_16;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_32)){rf_div = RF_DIV_32;}
	  else if(freq_khz>(MIN_VCO_FREQ_KHZ>>RF_DIV_64)){rf_div = RF_DIV_64;}
	  else {return -2;} //not in range (below)

	  //this will now all be done by dividing by 2 to keep within our desired range
	  //now we can get our pfd value divided by our RF divider (still in hz)
	  //RF_out = [INT_div_fact + (FRAC/MOD)]*(f_pfd/RF_div)
	  uint32_t f_pfd_rf_div_hz = f_pfd_hz>>rf_div; //first get our divided down f_pfd
	  //uint32_t f_pfd_rf_div_khz = f_pfd_rf_div_hz/1000; //same thing in khz
	  //make our divided by 2 frequency 
	  uint32_t freq_hz_div_2 = ((freq_khz_in>>1)*1000)+(freq_hz_in>>2);
	  uint32_t my_int = (freq_hz_div_2/(f_pfd_rf_div_hz))<<1; //calculate our int value (multiply by 2 to return to correct freq_hz)
	  uint32_t my_res_hz = (freq_hz_div_2-((my_int*f_pfd_rf_div_hz)>>1))<<1; //whats left in hz (some divided by 2 rounding to allow full range)
	  uint32_t my_mod = this->get_mod(); //get the closest value from the current step value
	  //gave up and used longer values... not great but fine for now
    uint32_t my_frac = (uint32_t)((uint64_t)(((uint64_t)my_mod)*((uint64_t)my_res_hz))/((uint64_t)f_pfd_rf_div_hz));

    
	  //now set the values in the corresponding data structures
	  //Serial.print("PFD_RF = "); Serial.println(f_pfd_rf_div_hz);
	  //Serial.print("PFD_RFK= "); Serial.println(f_pfd_rf_div_khz);
	  //Serial.print("FREQ_K = "); Serial.println(freq_khz);
	  //Serial.print("MOD    = "); Serial.println(my_mod);
	  //Serial.print("RES    = "); Serial.println(my_res_hz);
	  //Serial.print("RF_DIV = "); Serial.println(rf_div);
	  //Serial.print(" INT   = "); Serial.println(my_int);
	  //Serial.print(" FRAC  = "); Serial.println(my_frac);
	  this->set_rf_divider_select(rf_div); //set our rf divider
	  this->set_int((uint16_t)my_int); //set integer value
	  this->set_frac((uint16_t)my_frac); //set frac
	  
	  //now write out all of our registers to the ADF board
   //doing weird stuff with the board so stop for now while debugging
	  this->write_all();
	  return 0;
  }
  
 
  
//set values above int size (we will start at 4g for easy usage
  
//set frequency step. Then return what the actual step is (only some are possible)
uint32_t ADF4351::set_step(uint32_t freq_step_hz){
	//calculate f_pfd
	uint8_t r_dub_val = this->get_reference_doubler();
	uint8_t r_div_val = this->get_rdiv2();
	uint32_t f_pfd_hz = (REF_FREQ_RAW*(1+r_dub_val))/(r_div_val*(1+REF_DIV_BY_2));
	//get our current rf div value
	uint8_t rf_div = this->get_rf_divider_select();
	uint32_t f_pfd_rf_div_hz = f_pfd_hz>>rf_div; //get our divided down f_pfd
	uint32_t f_pfd_rf_div_khz = f_pfd_rf_div_hz/1000; //same thing in khz
	uint32_t my_mod = f_pfd_rf_div_hz/freq_step_hz;
	this->set_mod(my_mod);
	return f_pfd_rf_div_khz/my_mod;
}

//currently just for arduino
int8_t ADF4351::print_info(){
	Serial.printf("Register Summary:\n");
	Serial.printf("  R0:\n");
	Serial.printf("      INT  :   %u\n",this->get_int() );
	Serial.printf("      FRAC :   %u\n",this->get_frac());
	Serial.printf("  R1:\n");
	Serial.printf("      MOD  :   %u\n",this->get_mod() );
	Serial.printf("      PHASE:   %u\n",this->get_phase());
	Serial.printf("  R2:\n");
	Serial.printf("      RCNT :   %u\n",this->get_r_counter());
	Serial.printf("      RDIV :   %u\n",this->get_rdiv2());
	Serial.printf("      RDUB :   %u\n",this->get_reference_doubler());
	Serial.printf("      MXOUT:   %u\n",this->get_muxout());
	Serial.printf("  R3:\n");
	Serial.printf("     CLKDIV:   %u\n",this->get_clock_divider_value());
	Serial.printf("  R4:\n");
	Serial.printf("     OUTPWR:   %d\n",this->get_output_power());
	Serial.printf("      RFEN :   %d\n",this->get_rf_output_enable());
  Serial.printf("      RFDIV:   %u\n",this->get_rf_divider_select());
	Serial.printf("  R5:\n");
}

int8_t ADF4351::write_all(){
	int8_t err_count = 0;
	for(int8_t i=5;i>-1;--i) {
	  err_count+=R[i].write(); //write regs 5-0 in that order
	}
 return 0;
}
  
ADF4351_Register::ADF4351_Register(){}; //dont do anything right now

//int8_t ADF4351_Register::init_spi(SpiBus spi_bus_in){
//	my_spi_bus = spi_bus_in;
//}
			
	
		//write a raw uint32 value to the register
int8_t ADF4351_Register::write_data(uint32_t data){ 
				//first put our data into the value property
				this->set(data);
				return this->write();  //now write out
		}
		
		//write whatever is in our stored value to the register
int8_t ADF4351_Register::write(){
			//flip to big endian on arduino
      uint32_t data = this->value;
			uint8_t out_arr[4];
			out_arr[0] = uint8_t((data>>24)&0xFF);
			out_arr[1] = uint8_t((data>>16)&0xFF);
			out_arr[2] = uint8_t((data>>8)&0xFF);
			out_arr[3] = uint8_t((data>>0)&0xFF);
			//now actually write
			digitalWrite(ADF4351_SS_PIN,LOW); //slave select low
			SPI.transfer(out_arr,4); //send the data
			digitalWrite(ADF4351_SS_PIN,HIGH);//slave select high
			//with spi bus implementation
			//my_spi_bus.write(ADF4351_SS_PIN,(uint8_t*)&(my_spi_bus->swap_endian(data)),4);
			this->written_flg = 1;
			return 0;
		}			
		
		//set register value
int8_t ADF4351_Register::set(uint32_t data){
			this->value = data;
			this->written_flg = 0;
			return 0;
		}
		
uint32_t ADF4351_Register::get(){return this->value;}
		
		//set certain num_bits starting at offset
		//check for too large of numbers here
int8_t ADF4351_Register::set_bits(uint32_t data, uint8_t num_bits, uint8_t offset){
			this->written_flg = 0; //set to not written;
			uint32_t mymask = (1<<num_bits)-1; //mask at the 0 position
			//some error checking
			if(offset+num_bits>32) return -1;//check the offset and the num_bits
			//Serial.println(mymask);
			//Serial.println(data);
			//Serial.println(~mymask);
			if(mymask<data) return -2;
			this->value = (this->value&(~(mymask<<offset)))|((data&mymask)<<offset); //mask current value to zero then or with new data
			return 0;
		}
		
uint32_t ADF4351_Register::get_bits(uint8_t num_bits, uint8_t offset){
			uint32_t mymask = (1<<num_bits)-1; //mask at the 0 position
			return ((this->value)>>offset)&mymask; //shift data over then mask out just our number of bits we want
}

		//getter for written_flg
uint8_t ADF4351_Register::get_written_flg(){return this->written_flg;}







//---------------------------------------------------------------------------------------------
// GETTERS AND SETTERS FOR ALL REGISTERS IN ADF4351 -------------------------------------------
//---------------------------------------------------------------------------------------------
	//----------------------------------------
	//getters for register values
	//----------------------------------------
	//register 0 
	uint16_t ADF4351::get_frac							(){return (uint16_t)R[0].get_bits(12, 3);} 			
	uint16_t ADF4351::get_int							(){return (uint16_t)R[0].get_bits(16,15);}
	//register 1                                                                       
	uint16_t ADF4351::get_mod							(){return (uint16_t)R[1].get_bits(12, 3);} 			
	uint16_t ADF4351::get_phase							(){return (uint16_t)R[1].get_bits(12,15);}
	uint8_t ADF4351::get_prescaler						(){return (uint8_t )R[1].get_bits(1 ,27);} 		
	uint8_t ADF4351::get_phase_adjust					(){return (uint8_t )R[1].get_bits(1 ,28);}
	//register 2                                                                       
	uint8_t ADF4351::get_counter_reset					(){return (uint8_t )R[2].get_bits(1 , 3);} 	
	uint8_t ADF4351::get_cp_threestate					(){return (uint8_t )R[2].get_bits(1 , 4);}
	uint8_t ADF4351::get_power_down						(){return (uint8_t )R[2].get_bits(1 , 5);} 		
	uint8_t ADF4351::get_pd_polarity					(){return (uint8_t )R[2].get_bits(1 , 6);}
	uint8_t ADF4351::get_ldp							(){return (uint8_t )R[2].get_bits(1 , 7);} 				
	uint8_t ADF4351::get_ldf							(){return (uint8_t )R[2].get_bits(1 , 8);}
	uint8_t ADF4351::get_charge_pump_current_settings	(){return (uint8_t )R[2].get_bits(4 , 9);} 	
	uint8_t ADF4351::get_double_buffer					(){return (uint8_t )R[2].get_bits(1 ,13);}
	uint16_t ADF4351::get_r_counter						(){return (uint16_t)R[2].get_bits(10,14);} 		
	uint8_t ADF4351::get_rdiv2							(){return (uint8_t )R[2].get_bits(1 ,24);}
	uint8_t ADF4351::get_reference_doubler				(){return (uint8_t )R[2].get_bits(1 ,25);}
	uint8_t ADF4351::get_muxout							(){return (uint8_t )R[2].get_bits(3 ,26);}
	uint8_t ADF4351::get_low_noise_and_low_spur_modes	(){return (uint8_t )R[2].get_bits(2 ,29);}
	//register 3                                                                       
	uint16_t ADF4351::get_clock_divider_value			(){return (uint16_t)R[3].get_bits(12, 3);} 
	uint8_t ADF4351::get_clk_div_mode					(){return (uint8_t )R[3].get_bits(2 ,15);}
	uint8_t ADF4351::get_csr							(){return (uint8_t )R[3].get_bits(1 ,18);} 				
	uint8_t ADF4351::get_charge_cancel					(){return (uint8_t )R[3].get_bits(1 ,21);}
	uint8_t ADF4351::get_abp							(){return (uint8_t )R[3].get_bits(1 ,22);} 				
	uint8_t ADF4351::get_band_select_clock_mode			(){return (uint8_t )R[3].get_bits(1 ,23);}
	//register 4                                                                       
	uint8_t ADF4351::get_output_power					(){return (uint8_t )R[4].get_bits(2 , 3);} 	
	uint8_t ADF4351::get_rf_output_enable				(){return (uint8_t )R[4].get_bits(1 , 5);}
	uint8_t ADF4351::get_aux_output_power				(){return (uint8_t )R[4].get_bits(2 , 6);} 
	uint8_t ADF4351::get_aux_output_enable				(){return (uint8_t )R[4].get_bits(1 , 8);}
	uint8_t ADF4351::get_aux_output_select				(){return (uint8_t )R[4].get_bits(1 , 9);}
	uint8_t ADF4351::get_vco_powerdown					(){return (uint8_t )R[4].get_bits(1 ,10);}
	uint8_t ADF4351::get_mtld							(){return (uint8_t )R[4].get_bits(1 ,11);}
	uint8_t ADF4351::get_band_select_clock_divider_value(){return (uint8_t )R[4].get_bits(8 ,12);}
	uint8_t ADF4351::get_rf_divider_select				(){return (uint8_t )R[4].get_bits(3 ,20);}
	uint8_t ADF4351::get_rf_feedback_select				(){return (uint8_t )R[4].get_bits(1 ,23);}
	//register 5                                                                       
	uint8_t ADF4351::get_ld_pin_mode					(){return (uint8_t )R[5].get_bits(2 ,22);}
	
	//----------------------------------------
	//setters for register values
	//----------------------------------------
	//register 0 
	int8_t ADF4351::set_frac							(uint16_t data){return R[0].set_bits((uint32_t)data,12, 3);} 			
	int8_t ADF4351::set_int								(uint16_t data){return R[0].set_bits((uint32_t)data,16,15);}
	//register 1                                                                                                  
	int8_t ADF4351::set_mod								(uint16_t data){return R[1].set_bits((uint32_t)data,12, 3);} 			
	int8_t ADF4351::set_phase							(uint16_t data){return R[1].set_bits((uint32_t)data,12,15);}
	int8_t ADF4351::set_prescaler						(uint8_t  data){return R[1].set_bits((uint32_t)data,1 ,27);} 		
	int8_t ADF4351::set_phase_adjust					(uint8_t  data){return R[1].set_bits((uint32_t)data,1 ,28);}
	//register 2                                                                                                  
	int8_t ADF4351::set_counter_reset					(uint8_t  data){return R[2].set_bits((uint32_t)data,1 , 3);} 	
	int8_t ADF4351::set_cp_threestate					(uint8_t  data){return R[2].set_bits((uint32_t)data,1 , 4);}
	int8_t ADF4351::set_power_down						(uint8_t  data){return R[2].set_bits((uint32_t)data,1 , 5);} 		
	int8_t ADF4351::set_pd_polarity						(uint8_t  data){return R[2].set_bits((uint32_t)data,1 , 6);}
	int8_t ADF4351::set_ldp								(uint8_t  data){return R[2].set_bits((uint32_t)data,1 , 7);} 				
	int8_t ADF4351::set_ldf								(uint8_t  data){return R[2].set_bits((uint32_t)data,1 , 8);}
	int8_t ADF4351::set_charge_pump_current_settings	(uint8_t  data){return R[2].set_bits((uint32_t)data,4 , 9);} 	
	int8_t ADF4351::set_double_buffer					(uint8_t  data){return R[2].set_bits((uint32_t)data,1 ,13);}
	int8_t ADF4351::set_r_counter						(uint16_t data){return R[2].set_bits((uint32_t)data,10,14);} 		
	int8_t ADF4351::set_rdiv2							(uint8_t  data){return R[2].set_bits((uint32_t)data,1 ,24);}
	int8_t ADF4351::set_reference_doubler				(uint8_t  data){return R[2].set_bits((uint32_t)data,1 ,25);}
	int8_t ADF4351::set_muxout							(uint8_t  data){return R[2].set_bits((uint32_t)data,3 ,26);}
	int8_t ADF4351::set_low_noise_and_low_spur_modes	(uint8_t  data){return R[2].set_bits((uint32_t)data,2 ,29);}
	//register 3                                                                                                  
	int8_t ADF4351::set_clock_divider_value				(uint16_t data){return R[3].set_bits((uint32_t)data,12, 3);} 
	int8_t ADF4351::set_clk_div_mode					(uint8_t  data){return R[3].set_bits((uint32_t)data,2 ,15);}
	int8_t ADF4351::set_csr								(uint8_t  data){return R[3].set_bits((uint32_t)data,1 ,18);} 				
	int8_t ADF4351::set_charge_cancel					(uint8_t  data){return R[3].set_bits((uint32_t)data,1 ,21);}
	int8_t ADF4351::set_abp								(uint8_t  data){return R[3].set_bits((uint32_t)data,1 ,22);} 				
	int8_t ADF4351::set_band_select_clock_mode			(uint8_t  data){return R[3].set_bits((uint32_t)data,1 ,23);}
	//register 4                                                                                                  
	int8_t ADF4351::set_output_power					(uint8_t  data){return R[4].set_bits((uint32_t)data,2 , 3);} 	
	int8_t ADF4351::set_rf_output_enable				(uint8_t  data){return R[4].set_bits((uint32_t)data,1 , 5);}
	int8_t ADF4351::set_aux_output_power				(uint8_t  data){return R[4].set_bits((uint32_t)data,2 , 6);} 
	int8_t ADF4351::set_aux_output_enable				(uint8_t  data){return R[4].set_bits((uint32_t)data,1 , 8);}
	int8_t ADF4351::set_aux_output_select				(uint8_t  data){return R[4].set_bits((uint32_t)data,1 , 9);}
	int8_t ADF4351::set_vco_powerdown					(uint8_t  data){return R[4].set_bits((uint32_t)data,1 ,10);}
	int8_t ADF4351::set_mtld							(uint8_t  data){return R[4].set_bits((uint32_t)data,1 ,11);}
	int8_t ADF4351::set_band_select_clock_divider_value	(uint8_t  data){return R[4].set_bits((uint32_t)data,8 ,12);}
	int8_t ADF4351::set_rf_divider_select				(uint8_t  data){return R[4].set_bits((uint32_t)data,3 ,20);}
	int8_t ADF4351::set_rf_feedback_select				(uint8_t  data){return R[4].set_bits((uint32_t)data,1 ,23);}
	//register 5                                                                                                  
	int8_t ADF4351::set_ld_pin_mode						(uint8_t  data){return R[5].set_bits((uint32_t)data,2 ,22);}





	
