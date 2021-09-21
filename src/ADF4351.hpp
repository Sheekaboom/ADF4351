/************************************************************************
 *  @file    ADF4351.hpp
 *  @author  Alec Weiss
 *  @date    9/2018 
 *  @version 1.0 
 * 
 *  
 *  @brief Class to control ADF4351 (currently with arduino)
 *
 *  @section Header file for library for Analog devices ADF4351
 *  
 *************************************************************************/

#ifndef AL_ADF4351_H
#define AL_ADF4351_H

#include <stdint.h>
#include <SPI.h>

//SS_PIN defined so we dont have to pass it everywhere for arduino
//#define SS_PIN 10
#define ADF4351_SS_PIN 10

//Default register values. Should output at 45MHz
#define R0_DEF 0x00399900
#define R1_DEF 0x0000FD01
#define R2_DEF 0x00004F42
#define R3_DEF 0x000004B3
#define R4_DEF 0x00EC8024
#define R5_DEF 0x00580005

//Frequency definfition
#define REF_FREQ_RAW     25000000 	// reference frequency in Hz
#define MAX_VCO_FREQ_KHZ 4400000	//Maximum VCO Frequency in KHz
#define MIN_VCO_FREQ_KHZ 2200000  //minimum undivided VCO freq in KHz


//mux vals
#define MUXOUT_TRISTATE 0
#define MUXOUT_DVDD 	1
#define MUXOUT_GND 		2
#define MUXOUT_RCOUNT 	3
#define MUXOUT_NDIV 	4
#define MUXOUT_ALD 		5
#define MUXOUT_DLD 		6 //digital lock detect

//RF divider values
#define RF_DIV_1 0
#define RF_DIV_2 1
#define RF_DIV_4 2
#define RF_DIV_8 3
#define RF_DIV_16 4
#define RF_DIV_32 5
#define RF_DIV_64 6
#define GET_RF_DIV_VAL(reg_val) 1<<reg_val; //get our division factor (1-64) from our register number (1-6)

//RF power values
#define RF_POW_m4 0
#define RF_POW_m1 1
#define RF_POW_p2 2
#define RF_POW_p5 3
#define RF_POW_MIN RF_POW_m4
#define RF_POW_MAX RF_POW_p5

#define CHECK_ERR(x) if(x!=0) return x;

//something to save for later
#define REF_DIV_BY_2 0

//adf register
class ADF4351_Register{
  private:
    //value in register stored in whatever the machine endian is. THis may need to be compensated for in write.
    //this also takes into account whatever the address for the register is in the 3 lowest bits
    uint32_t value=0; 
    uint8_t  written_flg=0; //has the current value been written
	
	//SpiBus my_spi_bus; //SPI bus for hardware abstraction
    
  public:
    ADF4351_Register(); //dont do anything right now
  
    //write a raw uint32 value to the register
    int8_t write_data(uint32_t data);
    
    //write whatever is in our stored value to the register
    int8_t write();
	
	//write all registers 5-0 in that order
	int8_t write_all();
    
    int8_t set(uint32_t data); 	//set register value
	uint32_t get();				//get register value
    
    //set certain num_bits starting at offset with data
    int8_t set_bits(uint32_t data, uint8_t num_bits, uint8_t offset);
	uint32_t get_bits(uint8_t num_bits, uint8_t offset); //get num_bits at offset (return as uint32)

    //getter for written_flg
    uint8_t get_written_flg();
	
	//initialize spi by passing SpiBus in
	//int8_t init_spi(SpiBus spi_bus_in);
};
  

class ADF4351{

private:

  uint8_t data_pin, clk_pin, ss_pin; //our spi pins
  
  ADF4351_Register R[6]; //build registers 1-6


public:
  
  //constructor
	ADF4351(uint8_t data_pin_in, uint8_t clk_pin_in);
	//
  
	int8_t init();//startup registers
	
	//set a frequency in hz. This currently limits us to 4.3ish GHz
	int8_t set_freq(uint32_t freq_hz);
	
	//set frequency summing both khz and hz to reach full 4.4GHz range
	int8_t set_freq_full(uint32_t freq_khz_in,uint32_t freq_hz_in);
	
	//set the step in khz
	//return what it was actually set to (cant be exact)
	uint32_t set_step(uint32_t freq_step_hz);

    //write all registers
    int8_t write_all();
	
	//print register info (just important ones)
	int8_t print_info();
	
	//----------------------------------------
	//getters for register values
	//----------------------------------------
	//register 0 
	uint16_t get_frac							(); 			
	uint16_t get_int							();
	//register 1
	uint16_t get_mod							(); 			
	uint16_t get_phase							();
	uint8_t get_prescaler						(); 		
	uint8_t get_phase_adjust					();
	//register 2
	uint8_t get_counter_reset					(); 	
	uint8_t get_cp_threestate					();
	uint8_t get_power_down						(); 		
	uint8_t get_pd_polarity						();
	uint8_t get_ldp								(); 				
	uint8_t get_ldf								();
	uint8_t get_charge_pump_current_settings	(); 	
	uint8_t get_double_buffer					();
	uint16_t get_r_counter						(); 		
	uint8_t get_rdiv2							();
	uint8_t get_reference_doubler				();
	uint8_t get_muxout							();
	uint8_t get_low_noise_and_low_spur_modes	();
	//register 3
	uint16_t get_clock_divider_value			(); 
	uint8_t get_clk_div_mode					();
	uint8_t get_csr								(); 				
	uint8_t get_charge_cancel					();
	uint8_t get_abp								(); 				
	uint8_t get_band_select_clock_mode			();
	//register 4
	uint8_t get_output_power					(); 	
	uint8_t get_rf_output_enable				();
	uint8_t get_aux_output_power				(); 
	uint8_t get_aux_output_enable				();
	uint8_t get_aux_output_select				();
	uint8_t get_vco_powerdown					();
	uint8_t get_mtld							();
	uint8_t get_band_select_clock_divider_value	();
	uint8_t get_rf_divider_select				();
	uint8_t get_rf_feedback_select				();
	//register 5
	uint8_t get_ld_pin_mode						();
	
	//----------------------------------------
	//setters for register values
	//----------------------------------------
	//register 0 
	int8_t set_frac								(uint16_t); 			
	int8_t set_int								(uint16_t);
	//register 1                                 
	int8_t set_mod								(uint16_t); 			
	int8_t set_phase							(uint16_t);
	int8_t set_prescaler						(uint8_t ); 		
	int8_t set_phase_adjust						(uint8_t );
	//register 2                                 
	int8_t set_counter_reset					(uint8_t ); 	
	int8_t set_cp_threestate					(uint8_t );
	int8_t set_power_down						(uint8_t ); 		
	int8_t set_pd_polarity						(uint8_t );
	int8_t set_ldp								(uint8_t ); 				
	int8_t set_ldf								(uint8_t );
	int8_t set_charge_pump_current_settings		(uint8_t ); 	
	int8_t set_double_buffer					(uint8_t );
	int8_t set_r_counter						(uint16_t); 		
	int8_t set_rdiv2							(uint8_t );
	int8_t set_reference_doubler				(uint8_t );
	int8_t set_muxout							(uint8_t );
	int8_t set_low_noise_and_low_spur_modes		(uint8_t );
	//register 3                                 
	int8_t set_clock_divider_value				(uint16_t); 
	int8_t set_clk_div_mode						(uint8_t );
	int8_t set_csr								(uint8_t ); 				
	int8_t set_charge_cancel					(uint8_t );
	int8_t set_abp								(uint8_t ); 				
	int8_t set_band_select_clock_mode			(uint8_t );
	//register 4                                 
	int8_t set_output_power						(uint8_t ); 	
	int8_t set_rf_output_enable					(uint8_t );
	int8_t set_aux_output_power					(uint8_t ); 
	int8_t set_aux_output_enable				(uint8_t );
	int8_t set_aux_output_select				(uint8_t );
	int8_t set_vco_powerdown					(uint8_t );
	int8_t set_mtld								(uint8_t );
	int8_t set_band_select_clock_divider_value	(uint8_t );
	int8_t set_rf_divider_select				(uint8_t );
	int8_t set_rf_feedback_select				(uint8_t );
	//register 5                                 
	int8_t set_ld_pin_mode						(uint8_t );
	
};

#endif // AL_ADF4351_H
