--+----------------------------------------------------------------------------
--| 
--| COPYRIGHT 2018 United States Air Force Academy All rights reserved.
--| 
--| United States Air Force Academy     __  _______ ___    _________ 
--| Dept of Electrical &               / / / / ___//   |  / ____/   |
--| Computer Engineering              / / / /\__ \/ /| | / /_  / /| |
--| 2354 Fairchild Drive Ste 2F6     / /_/ /___/ / ___ |/ __/ / ___ |
--| USAF Academy, CO 80840           \____//____/_/  |_/_/   /_/  |_|
--| 
--| ---------------------------------------------------------------------------
--|
--| FILENAME      : top_basys3.vhd
--| AUTHOR(S)     : Capt Phillip Warner
--| CREATED       : 3/9/2018  MOdified by Capt Dan Johnson (3/30/2020)
--| DESCRIPTION   : This file implements the top level module for a BASYS 3 to 
--|					drive the Lab 4 Design Project (Advanced Elevator Controller).
--|
--|					Inputs: clk       --> 100 MHz clock from FPGA
--|							btnL      --> Rst Clk
--|							btnR      --> Rst FSM
--|							btnU      --> Rst Master
--|							btnC      --> GO (request floor)
--|							sw(15:12) --> Passenger location (floor select bits)
--| 						sw(3:0)   --> Desired location (floor select bits)
--| 						 - Minumum FUNCTIONALITY ONLY: sw(1) --> up_down, sw(0) --> stop
--|							 
--|					Outputs: led --> indicates elevator movement with sweeping pattern (additional functionality)
--|							   - led(10) --> led(15) = MOVING UP
--|							   - led(5)  --> led(0)  = MOVING DOWN
--|							   - ALL OFF		     = NOT MOVING
--|							 an(3:0)    --> seven-segment display anode active-low enable (AN3 ... AN0)
--|							 seg(6:0)	--> seven-segment display cathodes (CG ... CA.  DP unused)
--|
--| DOCUMENTATION : None
--|
--+----------------------------------------------------------------------------
--|
--| REQUIRED FILES :
--|
--|    Libraries : ieee
--|    Packages  : std_logic_1164, numeric_std
--|    Files     : MooreElevatorController.vhd, clock_divider.vhd, sevenSegDecoder.vhd
--|				   thunderbird_fsm.vhd, sevenSegDecoder, TDM4.vhd, OTHERS???
--|
--+----------------------------------------------------------------------------
--|
--| NAMING CONVENSIONS :
--|
--|    xb_<port name>           = off-chip bidirectional port ( _pads file )
--|    xi_<port name>           = off-chip input port         ( _pads file )
--|    xo_<port name>           = off-chip output port        ( _pads file )
--|    b_<port name>            = on-chip bidirectional port
--|    i_<port name>            = on-chip input port
--|    o_<port name>            = on-chip output port
--|    c_<signal name>          = combinatorial signal
--|    f_<signal name>          = synchronous signal
--|    ff_<signal name>         = pipeline stage (ff_, fff_, etc.)
--|    <signal name>_n          = active low signal
--|    w_<signal name>          = top level wiring signal
--|    g_<generic name>         = generic
--|    k_<constant name>        = constant
--|    v_<variable name>        = variable
--|    sm_<state machine type>  = state machine type definition
--|    s_<signal name>          = state name
--|
--+----------------------------------------------------------------------------
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;


-- Lab 4
entity top_basys3 is
    port(
        -- inputs
        clk     :   in std_logic; -- native 100MHz FPGA clock
        sw      :   in std_logic_vector(15 downto 0);
        btnU    :   in std_logic; -- master_reset
        btnL    :   in std_logic; -- clk_reset
        btnR    :   in std_logic; -- fsm_reset
        
        -- outputs
        led :   out std_logic_vector(15 downto 0);
        -- 7-segment display segments (active-low cathodes)
        seg :   out std_logic_vector(6 downto 0);
        -- 7-segment display active-low enables (anodes)
        an  :   out std_logic_vector(3 downto 0)
    );
end top_basys3;

architecture top_basys3_arch of top_basys3 is 
  
  
	-- declare components and signals
	 component clock_divider is
           generic ( constant k_DIV : natural := 2  );
           port (
               i_clk   : in std_logic;
               i_reset : in std_logic;
               o_clk   : out std_logic
           );
       end component clock_divider;
       
       component elevator_controller_fsm is
           port (
               i_clk     : in std_logic;
               i_reset   : in std_logic;
               i_up_down : in std_logic;
               i_stop    : in std_logic;
               o_floor   : out std_logic_vector(3 downto 0)
           );
       end component elevator_controller_fsm;
       
       component sevenSegDecoder is
           port (
               i_D   : in std_logic_vector(3 downto 0);
               o_S   : out std_logic_vector(6 downto 0)
           );
       end component sevenSegDecoder;
	
	
    signal w_clk : std_logic;
    signal w_floor : std_logic_vector(3 downto 0);

begin
    -- Instantiate the clock divider
    clock_divider_Inst: clock_divider
    generic map ( k_DIV => 50000000 )
    port map(
        i_reset => btnL or btnU,
        i_clk => clk,
        o_clk => w_clk
    );

    -- Instantiate the FSM
    fsm: elevator_controller_fsm port map(
        i_clk    => w_clk,
        i_reset  => btnR or btnU,
        i_up_down=> sw(1),
        i_stop   => sw(0),
        o_floor  => w_floor
    );

    -- Instantiate the 7-segment decoder
    seg_decoder: sevenSegDecoder port map(
        i_D     => w_floor,
        o_S     => seg
    );

    -- LED and 7-segment display anodes control (assuming additional functionality)
    -- Add code here to drive the `led` and `an` signals based on the design requirements.
  
    -- Ground unused LEDs if necessary
    led <= (
           15 => w_clk,
           11 => w_floor(0),
           12 => w_floor(1),
           13 => w_floor(2),
           14 => w_floor(3),
           others => '0' 
           );
    -- Control the 7-segment display anodes (assuming one display)
    an <= "1011"; -- Activates only the first 7-segment display (AN0)

end top_basys3_arch;
