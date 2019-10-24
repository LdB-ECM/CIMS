LIBRARY IEEE;   USE ieee.std_logic_1164.all;
                USE ieee.std_logic_arith.all;
                USE ieee.std_logic_unsigned.all;


ENTITY cimswork IS
   GENERIC (maxcount: INTEGER := 624;
                 deadtime: INTEGER := 50);
   PORT (
      --[ RESET SIGNAL INPUT ACTIVE LOW ]--
      reset: IN STD_LOGIC;

      --[ OSCILLATOR INPUTS ]--
      clk: IN STD_LOGIC;

      --[ PWM TEST OUTPUTS ]--
      pwm_clk_a: BUFFER STD_LOGIC;
      pwm_clk_b: BUFFER STD_LOGIC;

      --[ RESOLUTION SWITCH INPUTS ]--
      sw: IN STD_LOGIC_VECTOR (4 downto 1);

      --[ SIGNAL SELECTION INPUTS ]--
      sel_x_inputs: IN STD_LOGIC;
      sel_y_inputs: IN STD_LOGIC;
      sel_z_inputs: IN STD_LOGIC;

      --[ STEP SIGNAL INPUTS ]--
      x_step: IN STD_LOGIC;
      y_step: IN STD_LOGIC;
      z_step: IN STD_LOGIC;
      opt_step: IN STD_LOGIC;

      --[ DIRECTION SIGNAL INPUTS ]--
      x_dir: IN STD_LOGIC;
      y_dir: IN STD_LOGIC;
      z_dir: IN STD_LOGIC;
      opt_dir: IN STD_LOGIC;

      --[ ENABLE SIGNAL INPUTS ]--
      x_enable: IN STD_LOGIC;
      y_enable: IN STD_LOGIC;
      z_enable: IN STD_LOGIC;
      opt_enable: IN STD_LOGIC;

      --[ FULL STEP RESET ]--
      opt_reset: IN STD_LOGIC;

      --[ CONTROL OUTPUTS ]--
      on_full_step: OUT STD_LOGIC;
      fault: OUT STD_LOGIC;
      overcurrent: OUT STD_LOGIC;
      power_down: OUT STD_LOGIC;

      --[ FET OUTPUTS ]--
      drv2a: OUT STD_LOGIC;
      drv1a: OUT STD_LOGIC;
      ndea: OUT STD_LOGIC;
      drv2b: OUT STD_LOGIC;
      drv1b: OUT STD_LOGIC;
      ndeb: OUT STD_LOGIC;

      --[ LED OUTPUTS ]--
      enable_led: OUT STD_LOGIC;
      step_led: BUFFER STD_LOGIC;

      --[ A TO D COMPARATOR INPUTS ]--
      ani1: IN STD_LOGIC;
      ani2: IN STD_LOGIC;

      --[ D TO A OUTPUTS ]--
      sine: OUT STD_LOGIC_VECTOR (7 downto 0);
      cosine: OUT STD_LOGIC_VECTOR (7 downto 0)
    );

END cimswork;

ARCHITECTURE behaviour OF cimswork IS

   --[ INPUT MUX CONTROL SIGNALS ]--
   SIGNAL step: STD_LOGIC;
   SIGNAL dir: STD_LOGIC;
   SIGNAL enable: STD_LOGIC;
   SIGNAL reset_int: STD_LOGIC;
   SIGNAL muxsel: STD_LOGIC_VECTOR (1 downto 0);

   --[ SWITCH RESOLUTION CONTROL SIGNALS ]--
   SIGNAL stepsize: INTEGER RANGE 1 TO 128;
   SIGNAL last_sw: STD_LOGIC_VECTOR (4 downto 1);
   SIGNAL maxstep: INTEGER RANGE 128 TO 255;

   --[ SINE/COSINE VALUE CONTROL SIGNALS ]--
   SIGNAL cos_le: STD_LOGIC;
   SIGNAL sin_le: STD_LOGIC;
   SIGNAL table_le: STD_LOGIC;
   SIGNAL table_inv: STD_LOGIC;
   SIGNAL tablepos: INTEGER RANGE 0 TO 255;
   SIGNAL sinetab:  STD_LOGIC_VECTOR (7 downto 0);

   --[ STEP POSITION CONTROL SIGNALS ]--
   SIGNAL position: STD_LOGIC_VECTOR(9 downto 0);
   SIGNAL invtemp:  STD_LOGIC_VECTOR (7 downto 0);
   SIGNAL quad: STD_LOGIC_VECTOR (1 downto 0);

   --[ STATE MACHINE SIGNALS ]--
   SIGNAL st1_twice: STD_LOGIC;
   TYPE state_values IS (st0, st1, st2, st3);
   SIGNAL pres_state: state_values;

   --[ STEP PULSE LATCH AND CONTROL SIGNALS ]--
   SIGNAL reset_step: STD_LOGIC;
   SIGNAL step_latch: STD_LOGIC;
   SIGNAL dir_latch: STD_LOGIC;

   --[ PWM CONTROL SIGNALS ]--
   SIGNAL pwm_clk: STD_LOGIC;
   SIGNAL clk_a: STD_LOGIC;
   SIGNAL clk_b: STD_LOGIC;
   SIGNAL phase_a_pwm: STD_LOGIC;
   SIGNAL phase_b_pwm: STD_LOGIC;
   SIGNAL pwm_count: STD_LOGIC_VECTOR (9 downto 0);

   --[ POWER DOWN COUNT SIGNALS ]--
   SIGNAL powercount: STD_LOGIC_VECTOR (14 downto 0);

BEGIN

   --------------------------------------------------------------------------
   -- SELECT INPUT SOURCES PROCESS
   --------------------------------------------------------------------------
   -- Putting a jumper on one select input selects those inputs for control.
   -- In no jumper is on then the opto-coupler inputs are the controls.
   --------------------------------------------------------------------------
   selinputs: PROCESS (sel_x_inputs, sel_y_inputs, sel_z_inputs)
        VARIABLE sel: STD_LOGIC_VECTOR(1 downto 0);
   BEGIN
      IF (sel_x_inputs = '1') THEN                    -- use x signals selected
         muxsel <= "00";                              -- mux selector to "00"
      ELSIF (sel_y_inputs = '1') THEN                 -- use y signals selected
         muxsel <= "01";                              -- mux selector to "01"
      ELSIF (sel_z_inputs = '1') THEN                 -- use z signals selected
         muxsel <= "10";                              -- mux selector to "10"
      ELSE                                            -- opto signals to be used
         muxsel <= "11";                              -- mux selector to "11"
      END IF;
   END PROCESS selinputs;

   --------------------------------------------------------------------------
   -- STEP INPUT MUX PROCESS
   --------------------------------------------------------------------------
   -- The selection mux decides which step line will be the step control.
   --------------------------------------------------------------------------
   step_mux: PROCESS (muxsel, x_step, y_step, z_step, opt_step)
   BEGIN
      CASE muxsel IS
         WHEN "00" => step <= x_step;                 -- use x step signal
         WHEN "01" => step <= y_step;                 -- use y step signal
         WHEN "10" => step <= z_step;                 -- use z step signal
         WHEN OTHERS => step <= opt_step;             -- use opto step signal
      END CASE;
   END PROCESS step_mux;

   --------------------------------------------------------------------------
   -- DIR INPUT MUX PROCESS
   --------------------------------------------------------------------------
   -- The selection mux decides which dir line will be the direction control.
   --------------------------------------------------------------------------
   dir_mux: PROCESS (muxsel, x_dir, y_dir, z_dir, opt_dir)
   BEGIN
      CASE muxsel IS
         WHEN "00" => dir <= x_dir;                   -- use x direction signal
         WHEN "01" => dir <= y_dir;                   -- use y direction signal
         WHEN "10" => dir <= z_dir;                   -- use z direction signal
         WHEN OTHERS => dir <= opt_dir;               -- use opto direction signal
      END CASE;
   END PROCESS dir_mux;

   --------------------------------------------------------------------------
   -- ENABLE INPUT MUX PROCESS
   --------------------------------------------------------------------------
   -- The selection mux decides which enable line will be the enable control.
   --------------------------------------------------------------------------
   enable_mux: PROCESS (muxsel, x_enable, y_enable, z_enable, opt_enable)
   BEGIN
      CASE muxsel IS
         WHEN "00" => enable <= x_enable;             -- use x enable signal
         WHEN "01" => enable <= y_enable;             -- use y enable signal
         WHEN "10" => enable <= z_enable;             -- use z enable signal
         WHEN OTHERS => enable <= opt_enable;         -- use opto enable signal
      END CASE;
   END PROCESS enable_mux;

   --------------------------------------------------------------------------
   -- UPDATE LAST SWITCH STATE PROCESS
   --------------------------------------------------------------------------
   -- The last switch state is update on each rising edge of the clock.
   --------------------------------------------------------------------------
   switch_update: PROCESS (clk, sw)
   BEGIN
      IF rising_edge(clk) THEN                        -- on each rising clock
         last_sw <= sw;                               -- update switch state
      END IF;
   END PROCESS switch_update;

   --------------------------------------------------------------------------
   -- RESET INPUT MUX PROCESS
   --------------------------------------------------------------------------
   -- The selection mux decides which reset lines will be the reset control.
   --------------------------------------------------------------------------
   reset_mux: PROCESS (muxsel, opt_reset, reset, sw, last_sw)
        VARIABLE reset_sw: STD_LOGIC;
   BEGIN
      IF (sw = last_sw) THEN                          -- if switches match
         reset_sw := reset;                           -- transpose hard reset
      ELSE                                            -- switches differ
         reset_sw := '0';                             -- reset required
      END IF;
      IF (muxsel = "11") THEN                         -- opto input selected
         reset_int <= reset_sw  AND opt_reset;        -- combine resets
      ELSE                                            -- none opto input
         reset_int <= reset_sw;                       -- transfer reset signal
      END IF;
   END PROCESS reset_mux;

   --------------------------------------------------------------------------
   -- PWM CLOCK SIGNAL PROCESS
   --------------------------------------------------------------------------
   -- This creates two pwm clocks which have a deadtime seperation between.
   --------------------------------------------------------------------------
   clock_pwm: PROCESS (reset, clk)
   BEGIN
      IF (reset = '0') THEN                           -- asynch power down reset
         pwm_count <= (OTHERS => '0');                -- reset counter to zero
         pwm_clk <= '0';                              -- set clock output low
         clk_a <= '0';                                -- clock a starts low
         clk_b <= '1';                                -- clock b starts high
      ELSIF rising_edge(clk) THEN                     -- rising edge of clock
         IF (pwm_count = maxcount) THEN
            pwm_count <= (OTHERS => '0');             -- reset counter to zero
            pwm_clk <= NOT pwm_clk;                   -- invert clock
            clk_a <= NOT pwm_clk;                     -- a follows pwm_clk
            clk_b <= pwm_clk;                         -- b invert of pwm_clk
         ELSE
            clk_a <= clk_a;                           -- latch clock a
            clk_b <= clk_b;                           -- latch clock b
            pwm_count <= pwm_count + 1;               -- increment counter
         END IF;
         IF (pwm_count = maxcount-deadtime) THEN
            clk_a <= '0';                             -- pwm phase a off
            clk_b <= '0';                             -- pwm phase b off
         END IF;
      END IF;
END PROCESS clock_pwm;

   --------------------------------------------------------------------------
   -- STEP POSITION PROCESS
   --------------------------------------------------------------------------
   -- Reset zeros the position counter. On each step pulse stepsize is added
   -- or subtracted to position as required.
   --------------------------------------------------------------------------
   step_position: PROCESS (reset_int, step, dir, position, stepsize, last_sw(4))
   BEGIN
      IF (reset_int = '0') THEN                       -- reset is active
         dir_latch <= '1';                            -- preset positive dir
         position <= (OTHERS => '0');                 -- clear position
      ELSIF rising_edge(step) THEN                    -- positive step edge
         dir_latch <= dir;                            -- latch direction
         IF (dir = '1') THEN
            position <= position + stepsize;
         ELSE
            position <= position - stepsize;
         END IF;
      END IF;
   END PROCESS step_position;

   --------------------------------------------------------------------------
   -- STEP LED OUTPUT PROCESS
   --------------------------------------------------------------------------
   -- This toggles the step led output on/off for each step pulse received.
   --------------------------------------------------------------------------
   step_led_out: PROCESS (reset_int, step)
   BEGIN
      IF (reset_int = '0') THEN                       -- reset is active
         step_led <= '0';                             -- step led starts off
      ELSIF rising_edge(step) THEN                    -- positive step edge
         step_led <= NOT step_led;                    -- change step led around
      END IF;
   END PROCESS step_led_out;

   --------------------------------------------------------------------------
   -- CREATE STEP PULSE SIGNAL PROCESS
   --------------------------------------------------------------------------
   -- Step going high sends this signal high until step_reset is asserted.
   --------------------------------------------------------------------------
   step_sig: PROCESS (reset_step, step)
   BEGIN
      IF (reset_step = '1') THEN                      -- asynch step reset
         step_latch <= '0';                           -- step pulse off
      ELSIF rising_edge(step) THEN                    -- positive step edge
         step_latch <= '1';                           -- set step pulse to high
      END IF;
   END PROCESS step_sig;

   --------------------------------------------------------------------------
   -- LATCH QUADRANT PROCESS
   --------------------------------------------------------------------------
   -- The step signal is asynchronous to the unit clock hence position may
   -- change asynchronous to us. The two high bits of position are the actual
   -- quadrants and as they are used by other sections they need to be latched
   -- on every step movement. At the same time on_full_step output is done.
   --------------------------------------------------------------------------
   latch_quadrant: PROCESS (reset_int, sin_le, position)
   BEGIN
      IF (reset_int = '0') THEN                       -- reset is active
         quad <= (OTHERS => '0');                     -- start on quadrant 1
         on_full_step <= '1';                         -- we start on full step
      ELSIF rising_edge(sin_le) THEN                  -- latch signal high
         quad(1) <= position(9);                      -- latch position bit 9
         quad(0) <= position(8);                      -- latch position bit 8
         IF (position(7 downto 0) = 0) THEN           -- 0,256,512,768 are full steps
            on_full_step <= '1';                      -- we are on full step
         ELSE
            on_full_step <= '0';                      -- not on full step
         END IF;
      END IF;
   END PROCESS latch_quadrant;

   --------------------------------------------------------------------------
   -- STATE MACHINE PROCESS
   --------------------------------------------------------------------------
   -- The state machine moves outputs sine/cosine wave as per table position.
   --------------------------------------------------------------------------
   fsm: PROCESS (reset_int, clk, pres_state, position(8))
   BEGIN
   IF (reset_int = '0') THEN                       -- reset is active
      pres_state <= st0;                           -- set initial state
      sin_le <= '0';                               -- sine latch low
      cos_le <= '0';                               -- cosine latch low
      reset_step <= '1';                           -- step reset at start up
   ELSIF rising_edge(clk) THEN                     -- positive clock edge
      sin_le <= '0';                               -- sine latch low
      cos_le <= '0';                               -- cosine latch low
      reset_step <= step_latch;                    -- follow step latch signal
      CASE pres_state IS
         WHEN st0 =>                               -- STATE 0
            pres_state <= st0;                     -- preset stay state 0
            IF (step_latch = '1') THEN             -- wait for step pulse
               st1_twice <= '0';                   -- must do st1 twice
               pres_state <= st1;                  -- move to next state
            END IF;
         WHEN st1 =>                               -- STATE 1
            IF (st1_twice = '0') THEN              -- first time of st1
               table_inv <= position(8);           -- set table invert state
               st1_twice <= '1';                   -- set st1 twice flag
               pres_state <= st1;                  -- stay on st1
            ELSE
               sin_le <= '1';                      -- latch sine output on low
               pres_state <= st2;                  -- move to next state
            END IF;
         WHEN st2 =>                               -- STATE 2 ( latch table position )
            table_inv <= NOT position(8);          -- set table invert state
            pres_state <= st3;                     -- move to next state
         WHEN st3 =>                               -- STATE 3
            cos_le <= '1';                         -- latch cosine output on low
            pres_state <= st0;                     -- move to initial state
         END CASE;
      END IF;
   END PROCESS fsm;

   --------------------------------------------------------------------------
   -- PHASE A OUTPUT PROCESS
   --------------------------------------------------------------------------
   -- PWM A is turned on on clock a edge and off if ref voltage exceeded.
   --------------------------------------------------------------------------
   phase_a_out: PROCESS (clk_a, ani1)
   BEGIN
      IF (ani1 = '1') THEN                            -- above reference
         phase_a_pwm <= '1';                          -- phase a off
      ELSIF rising_edge(clk_a) THEN                   -- time to turn phase on
         phase_a_pwm <= '0';                          -- phase a turned on
      END IF;
   END PROCESS phase_a_out;

   --------------------------------------------------------------------------
   -- PHASE B OUTPUT PROCESS
   --------------------------------------------------------------------------
   -- PWM B is turned on on clock a edge and off if ref voltage exceeded.
   --------------------------------------------------------------------------
   phase_b_out: PROCESS (clk_b, ani2)
   BEGIN
      IF (ani2 = '1') THEN                            -- above reference
         phase_b_pwm <= '1';                          -- phase b off
      ELSIF rising_edge(clk_b) THEN                   -- time to turm phase on
         phase_b_pwm <= '0';                          -- phase b turned on
      END IF;
   END PROCESS phase_b_out;

   --------------------------------------------------------------------------
   -- PHASE A OUTPUT SIGNALS PROCESS
   --------------------------------------------------------------------------
   -- Depending on quadrant phase a signal switches side to side.
   --------------------------------------------------------------------------
   phase_a_signals: PROCESS (quad(1), phase_a_pwm)
   BEGIN
      IF (quad(1) = '0') THEN                         -- in quad 1 or quad 2
         drv2a <= '1';                                -- hold signal high
         drv1a <= phase_a_pwm;                        -- reflect pwm signal
      ELSE                                            -- in quad 3 or quad 4
         drv2a <= phase_a_pwm;                        -- reflect pwm signal
         drv1a <= '1';                                -- hold signal high
      END IF;
   END PROCESS phase_a_signals;

   --------------------------------------------------------------------------
   -- PHASE B OUTPUT SIGNALS PROCESS
   --------------------------------------------------------------------------
   -- Depending on quadrant phase b signal switches side to side.
   --------------------------------------------------------------------------
   phase_b_signals: PROCESS (quad, phase_b_pwm)
   BEGIN
      IF ((quad(1) XOR quad(0)) = '0') THEN           -- in quad 1 or quad 4
         drv2b <= '1';                                -- hold signal high
         drv1b <= phase_b_pwm;                        -- reflect pwm signal
      ELSE                                            -- in quad 2 or quad 3
         drv2b <= phase_b_pwm;                        -- reflect pwm signal
         drv1b <= '1';                                -- hold signal high
      END IF;
   END PROCESS phase_b_signals;

   --------------------------------------------------------------------------
   -- DRIVE ENABLE OUTPUT SIGNALS PROCESS
   --------------------------------------------------------------------------
   -- This creates the enable signals which besides following the enable input
   -- must control the motor current decay between fast and slow decay as
   -- determined by the phase of the motor and speed. It is the most important
   -- part of the preformance of the motor unit.
   --------------------------------------------------------------------------
   drive_enable: PROCESS (reset_int, enable, dir_latch, quad(0), phase_a_pwm, phase_b_pwm, clk_a, clk_b)
   VARIABLE tempdir: STD_LOGIC;
   BEGIN
      IF (reset_int = '0') THEN                       -- reset is active
         ndea <= '0';                                 -- ensure drive A off
         ndeb <= '0';                                 -- ensure drive B off
         enable_led <= '0';                           -- enable led off
      ELSE                                            -- not in reset
         tempdir := quad(0) XOR dir_latch;            -- create temp dir
         IF (enable = '0') OR (tempdir = '1') THEN    -- slow decay quadrant
            ndea <= enable;                           -- follow enable signal
         ELSE
            ndea <= phase_b_pwm OR (NOT clk_b);       -- fast decay quadrant
         END IF;
         IF (enable = '0') OR (tempdir = '0') THEN    -- slow decay quadrant
            ndeb <= enable;                           -- follow enable signal
         ELSE
            ndeb <= phase_a_pwm OR (NOT clk_a);       -- fast decay quadrant
         END IF;
         enable_led <= enable;                        -- led follows enable signal
      END IF;
   END PROCESS drive_enable;

   --------------------------------------------------------------------------
   -- POWER DOWN OUTPUT SIGNALS PROCESS
   --------------------------------------------------------------------------
   -- If no step pulses are received in 1 second turns power down on. On the
   -- presence of a step pulse power down is immediately removed.
   --------------------------------------------------------------------------
   pwrdwn_output: PROCESS (step_latch, pwm_clk)
   BEGIN
      IF (step_latch = '1') THEN                      -- asynch power down reset
         power_down <= '0';                           -- turn power down off
         powercount <= (OTHERS => '0');               -- zero power down count
      ELSIF rising_edge(pwm_clk) THEN                 -- rising edge of clock
         IF (powercount = 32767) THEN                 -- approx 1 second up
            power_down <= '1';                        -- turn power down on
         ELSE
            powercount <= powercount + 1;             -- count pwm clock pulses
         END IF;
      END IF;
   END PROCESS pwrdwn_output;

   --------------------------------------------------------------------------
   -- FAULT OUTPUT SIGNALS PROCESS
   --------------------------------------------------------------------------
   fault_output: PROCESS (reset)
   BEGIN
       fault <= '0';
   END PROCESS fault_output;

   --------------------------------------------------------------------------
   -- OVERCURRENT OUTPUT SIGNALS PROCESS
   --------------------------------------------------------------------------
   over_current_output: PROCESS (reset)
   BEGIN
      overcurrent <= '0';
   END PROCESS over_current_output;

   --------------------------------------------------------------------------
   -- OUTPUT TEST SIGNALS PROCESS
   --------------------------------------------------------------------------
   test_signals: PROCESS (phase_a_pwm, phase_b_pwm)
   BEGIN
      pwm_clk_a <= phase_a_pwm;
      pwm_clk_b <= phase_b_pwm;
   END PROCESS test_signals;

   --------------------------------------------------------------------------
   -- INVERT POSITION (7 DOWNTO 0) IF NEEDED PROCESS
   --------------------------------------------------------------------------
   -- position(7 downto 0) is inverted or not depending on table_inv state
   --------------------------------------------------------------------------
   ctrl_pos_inv: PROCESS (position(7 downto 0), table_inv)
   BEGIN
      IF (table_inv = '1') THEN                       -- is table invert active
         invtemp <= (NOT position (7 downto 0));      -- set invert table position
      ELSE                                            -- invert not active
         invtemp <= position(7 downto 0);             -- set table position
      END IF;
   END PROCESS ctrl_pos_inv;

   --------------------------------------------------------------------------
   -- COMBINE LATCH PULSES PROCESS
   --------------------------------------------------------------------------
   -- The table_le output is created from the sin_le and cos_le signals.
   --------------------------------------------------------------------------
   comb_latch_pulses: PROCESS (sin_le, cos_le)
   BEGIN
      table_le <= sin_le OR cos_le;                   -- or latch signals
   END PROCESS comb_latch_pulses;

   --------------------------------------------------------------------------
   -- TABLE POSITION LATCH PROCESS
   --------------------------------------------------------------------------
   -- The tablepos output is latched on rising edge of table_le signal.
   --------------------------------------------------------------------------
   table_latch: PROCESS (table_le, invtemp)
   BEGIN
      IF rising_edge(table_le) THEN                   -- on table latch high edge
         tablepos <= CONV_INTEGER(invtemp);           -- latch value
      END IF;
   END PROCESS table_latch;

   --------------------------------------------------------------------------
   -- SINE TABLE DATA PROCESS
   --------------------------------------------------------------------------
   -- This is the sine table for 0 to 90 degree in 256 steps.
   --------------------------------------------------------------------------
   sine_table: PROCESS (tablepos)
   BEGIN
      CASE tablepos IS
         WHEN 0 => sinetab <= "00000000";             -- 0
         WHEN 1 => sinetab <= "00000010";             -- 2
         WHEN 2 => sinetab <= "00000011";             -- 3
         WHEN 3 => sinetab <= "00000101";             -- 5
         WHEN 4 => sinetab <= "00000110";             -- 6
         WHEN 5 => sinetab <= "00001000";             -- 8
         WHEN 6 => sinetab <= "00001001";             -- 9
         WHEN 7 => sinetab <= "00001011";             -- 11
         WHEN 8 => sinetab <= "00001101";             -- 13
         WHEN 9 => sinetab <= "00001110";             -- 14
         WHEN 10 => sinetab <= "00010000";            -- 16
         WHEN 11 => sinetab <= "00010001";            -- 17
         WHEN 12 => sinetab <= "00010011";            -- 19
         WHEN 13 => sinetab <= "00010100";            -- 20
         WHEN 14 => sinetab <= "00010110";            -- 22
         WHEN 15 => sinetab <= "00010111";            -- 23
         WHEN 16 => sinetab <= "00011001";            -- 25
         WHEN 17 => sinetab <= "00011011";            -- 27
         WHEN 18 => sinetab <= "00011100";            -- 28
         WHEN 19 => sinetab <= "00011110";            -- 30
         WHEN 20 => sinetab <= "00011111";            -- 31
         WHEN 21 => sinetab <= "00100001";            -- 33
         WHEN 22 => sinetab <= "00100010";            -- 34
         WHEN 23 => sinetab <= "00100100";            -- 36
         WHEN 24 => sinetab <= "00100101";            -- 37
         WHEN 25 => sinetab <= "00100111";            -- 39
         WHEN 26 => sinetab <= "00101001";            -- 41
         WHEN 27 => sinetab <= "00101010";            -- 42
         WHEN 28 => sinetab <= "00101100";            -- 44
         WHEN 29 => sinetab <= "00101101";            -- 45
         WHEN 30 => sinetab <= "00101111";            -- 47
         WHEN 31 => sinetab <= "00110000";            -- 48
         WHEN 32 => sinetab <= "00110010";            -- 50
         WHEN 33 => sinetab <= "00110011";            -- 51
         WHEN 34 => sinetab <= "00110101";            -- 53
         WHEN 35 => sinetab <= "00110110";            -- 54
         WHEN 36 => sinetab <= "00111000";            -- 56
         WHEN 37 => sinetab <= "00111001";            -- 57
         WHEN 38 => sinetab <= "00111011";            -- 59
         WHEN 39 => sinetab <= "00111100";            -- 60
         WHEN 40 => sinetab <= "00111110";            -- 62
         WHEN 41 => sinetab <= "00111111";            -- 63
         WHEN 42 => sinetab <= "01000001";            -- 65
         WHEN 43 => sinetab <= "01000011";            -- 67
         WHEN 44 => sinetab <= "01000100";            -- 68
         WHEN 45 => sinetab <= "01000110";            -- 70
         WHEN 46 => sinetab <= "01000111";            -- 71
         WHEN 47 => sinetab <= "01001001";            -- 73
         WHEN 48 => sinetab <= "01001010";            -- 74
         WHEN 49 => sinetab <= "01001100";            -- 76
         WHEN 50 => sinetab <= "01001101";            -- 77
         WHEN 51 => sinetab <= "01001111";            -- 79
         WHEN 52 => sinetab <= "01010000";            -- 80
         WHEN 53 => sinetab <= "01010001";            -- 81
         WHEN 54 => sinetab <= "01010011";            -- 83
         WHEN 55 => sinetab <= "01010100";            -- 84
         WHEN 56 => sinetab <= "01010110";            -- 86
         WHEN 57 => sinetab <= "01010111";            -- 87
         WHEN 58 => sinetab <= "01011001";            -- 89
         WHEN 59 => sinetab <= "01011010";            -- 90
         WHEN 60 => sinetab <= "01011100";            -- 92
         WHEN 61 => sinetab <= "01011101";            -- 93
         WHEN 62 => sinetab <= "01011111";            -- 95
         WHEN 63 => sinetab <= "01100000";            -- 96
         WHEN 64 => sinetab <= "01100010";            -- 98
         WHEN 65 => sinetab <= "01100011";            -- 99
         WHEN 66 => sinetab <= "01100100";            -- 100
         WHEN 67 => sinetab <= "01100110";            -- 102
         WHEN 68 => sinetab <= "01100111";            -- 103
         WHEN 69 => sinetab <= "01101001";            -- 105
         WHEN 70 => sinetab <= "01101010";            -- 106
         WHEN 71 => sinetab <= "01101100";            -- 108
         WHEN 72 => sinetab <= "01101101";            -- 109
         WHEN 73 => sinetab <= "01101110";            -- 110
         WHEN 74 => sinetab <= "01110000";            -- 112
         WHEN 75 => sinetab <= "01110001";            -- 113
         WHEN 76 => sinetab <= "01110011";            -- 115
         WHEN 77 => sinetab <= "01110100";            -- 116
         WHEN 78 => sinetab <= "01110101";            -- 117
         WHEN 79 => sinetab <= "01110111";            -- 119
         WHEN 80 => sinetab <= "01111000";            -- 120
         WHEN 81 => sinetab <= "01111010";            -- 122
         WHEN 82 => sinetab <= "01111011";            -- 123
         WHEN 83 => sinetab <= "01111100";            -- 124
         WHEN 84 => sinetab <= "01111110";            -- 126
         WHEN 85 => sinetab <= "01111111";            -- 127
         WHEN 86 => sinetab <= "10000000";            -- 128
         WHEN 87 => sinetab <= "10000010";            -- 130
         WHEN 88 => sinetab <= "10000011";            -- 131
         WHEN 89 => sinetab <= "10000100";            -- 132
         WHEN 90 => sinetab <= "10000110";            -- 134
         WHEN 91 => sinetab <= "10000111";            -- 135
         WHEN 92 => sinetab <= "10001000";            -- 136
         WHEN 93 => sinetab <= "10001010";            -- 138
         WHEN 94 => sinetab <= "10001011";            -- 139
         WHEN 95 => sinetab <= "10001100";            -- 140
         WHEN 96 => sinetab <= "10001110";            -- 142
         WHEN 97 => sinetab <= "10001111";            -- 143
         WHEN 98 => sinetab <= "10010000";            -- 144
         WHEN 99 => sinetab <= "10010010";            -- 146
         WHEN 100 => sinetab <= "10010011";           -- 147
         WHEN 101 => sinetab <= "10010100";           -- 148
         WHEN 102 => sinetab <= "10010101";           -- 149
         WHEN 103 => sinetab <= "10010111";           -- 151
         WHEN 104 => sinetab <= "10011000";           -- 152
         WHEN 105 => sinetab <= "10011001";           -- 153
         WHEN 106 => sinetab <= "10011010";           -- 154
         WHEN 107 => sinetab <= "10011100";           -- 156
         WHEN 108 => sinetab <= "10011101";           -- 157
         WHEN 109 => sinetab <= "10011110";           -- 158
         WHEN 110 => sinetab <= "10011111";           -- 159
         WHEN 111 => sinetab <= "10100001";           -- 161
         WHEN 112 => sinetab <= "10100010";           -- 162
         WHEN 113 => sinetab <= "10100011";           -- 163
         WHEN 114 => sinetab <= "10100100";           -- 164
         WHEN 115 => sinetab <= "10100101";           -- 165
         WHEN 116 => sinetab <= "10100111";           -- 167
         WHEN 117 => sinetab <= "10101000";           -- 168
         WHEN 118 => sinetab <= "10101001";           -- 169
         WHEN 119 => sinetab <= "10101010";           -- 170
         WHEN 120 => sinetab <= "10101011";           -- 171
         WHEN 121 => sinetab <= "10101100";           -- 172
         WHEN 122 => sinetab <= "10101110";           -- 174
         WHEN 123 => sinetab <= "10101111";           -- 175
         WHEN 124 => sinetab <= "10110000";           -- 176
         WHEN 125 => sinetab <= "10110001";           -- 177
         WHEN 126 => sinetab <= "10110010";           -- 178
         WHEN 127 => sinetab <= "10110011";           -- 179
         WHEN 128 => sinetab <= "10110100";           -- 180
         WHEN 129 => sinetab <= "10110101";           -- 181
         WHEN 130 => sinetab <= "10110111";           -- 183
         WHEN 131 => sinetab <= "10111000";           -- 184
         WHEN 132 => sinetab <= "10111001";           -- 185
         WHEN 133 => sinetab <= "10111010";           -- 186
         WHEN 134 => sinetab <= "10111011";           -- 187
         WHEN 135 => sinetab <= "10111100";           -- 188
         WHEN 136 => sinetab <= "10111101";           -- 189
         WHEN 137 => sinetab <= "10111110";           -- 190
         WHEN 138 => sinetab <= "10111111";           -- 191
         WHEN 139 => sinetab <= "11000000";           -- 192
         WHEN 140 => sinetab <= "11000001";           -- 193
         WHEN 141 => sinetab <= "11000010";           -- 194
         WHEN 142 => sinetab <= "11000011";           -- 195
         WHEN 143 => sinetab <= "11000100";           -- 196
         WHEN 144 => sinetab <= "11000101";           -- 197
         WHEN 145 => sinetab <= "11000110";           -- 198
         WHEN 146 => sinetab <= "11000111";           -- 199
         WHEN 147 => sinetab <= "11001000";           -- 200
         WHEN 148 => sinetab <= "11001001";           -- 201
         WHEN 149 => sinetab <= "11001010";           -- 202
         WHEN 150 => sinetab <= "11001011";           -- 203
         WHEN 151 => sinetab <= "11001100";           -- 204
         WHEN 152 => sinetab <= "11001101";           -- 205
         WHEN 153 => sinetab <= "11001110";           -- 206
         WHEN 154 => sinetab <= "11001111";           -- 207
         WHEN 155 => sinetab <= "11010000";           -- 208
         WHEN 156 => sinetab <= "11010000";           -- 208
         WHEN 157 => sinetab <= "11010001";           -- 209
         WHEN 158 => sinetab <= "11010010";           -- 210
         WHEN 159 => sinetab <= "11010011";           -- 211
         WHEN 160 => sinetab <= "11010100";           -- 212
         WHEN 161 => sinetab <= "11010101";           -- 213
         WHEN 162 => sinetab <= "11010110";           -- 214
         WHEN 163 => sinetab <= "11010111";           -- 215
         WHEN 164 => sinetab <= "11010111";           -- 215
         WHEN 165 => sinetab <= "11011000";           -- 216
         WHEN 166 => sinetab <= "11011001";           -- 217
         WHEN 167 => sinetab <= "11011010";           -- 218
         WHEN 168 => sinetab <= "11011011";           -- 219
         WHEN 169 => sinetab <= "11011100";           -- 220
         WHEN 170 => sinetab <= "11011100";           -- 220
         WHEN 171 => sinetab <= "11011101";           -- 221
         WHEN 172 => sinetab <= "11011110";           -- 222
         WHEN 173 => sinetab <= "11011111";           -- 223
         WHEN 174 => sinetab <= "11011111";           -- 223
         WHEN 175 => sinetab <= "11100000";           -- 224
         WHEN 176 => sinetab <= "11100001";           -- 225
         WHEN 177 => sinetab <= "11100010";           -- 226
         WHEN 178 => sinetab <= "11100010";           -- 226
         WHEN 179 => sinetab <= "11100011";           -- 227
         WHEN 180 => sinetab <= "11100100";           -- 228
         WHEN 181 => sinetab <= "11100100";           -- 228
         WHEN 182 => sinetab <= "11100101";           -- 229
         WHEN 183 => sinetab <= "11100110";           -- 230
         WHEN 184 => sinetab <= "11100111";           -- 231
         WHEN 185 => sinetab <= "11100111";           -- 231
         WHEN 186 => sinetab <= "11101000";           -- 232
         WHEN 187 => sinetab <= "11101000";           -- 232
         WHEN 188 => sinetab <= "11101001";           -- 233
         WHEN 189 => sinetab <= "11101010";           -- 234
         WHEN 190 => sinetab <= "11101010";           -- 234
         WHEN 191 => sinetab <= "11101011";           -- 235
         WHEN 192 => sinetab <= "11101100";           -- 236
         WHEN 193 => sinetab <= "11101100";           -- 236
         WHEN 194 => sinetab <= "11101101";           -- 237
         WHEN 195 => sinetab <= "11101101";           -- 237
         WHEN 196 => sinetab <= "11101110";           -- 238
         WHEN 197 => sinetab <= "11101110";           -- 238
         WHEN 198 => sinetab <= "11101111";           -- 239
         WHEN 199 => sinetab <= "11110000";           -- 240
         WHEN 200 => sinetab <= "11110000";           -- 240
         WHEN 201 => sinetab <= "11110001";           -- 241
         WHEN 202 => sinetab <= "11110001";           -- 241
         WHEN 203 => sinetab <= "11110010";           -- 242
         WHEN 204 => sinetab <= "11110010";           -- 242
         WHEN 205 => sinetab <= "11110011";           -- 243
         WHEN 206 => sinetab <= "11110011";           -- 243
         WHEN 207 => sinetab <= "11110100";           -- 244
         WHEN 208 => sinetab <= "11110100";           -- 244
         WHEN 209 => sinetab <= "11110100";           -- 244
         WHEN 210 => sinetab <= "11110101";           -- 245
         WHEN 211 => sinetab <= "11110101";           -- 245
         WHEN 212 => sinetab <= "11110110";           -- 246
         WHEN 213 => sinetab <= "11110110";           -- 246
         WHEN 214 => sinetab <= "11110111";           -- 247
         WHEN 215 => sinetab <= "11110111";           -- 247
         WHEN 216 => sinetab <= "11110111";           -- 247
         WHEN 217 => sinetab <= "11111000";           -- 248
         WHEN 218 => sinetab <= "11111000";           -- 248
         WHEN 219 => sinetab <= "11111000";           -- 248
         WHEN 220 => sinetab <= "11111001";           -- 249
         WHEN 221 => sinetab <= "11111001";           -- 249
         WHEN 222 => sinetab <= "11111001";           -- 249
         WHEN 223 => sinetab <= "11111010";           -- 250
         WHEN 224 => sinetab <= "11111010";           -- 250
         WHEN 225 => sinetab <= "11111010";           -- 250
         WHEN 226 => sinetab <= "11111011";           -- 251
         WHEN 227 => sinetab <= "11111011";           -- 251
         WHEN 228 => sinetab <= "11111011";           -- 251
         WHEN 229 => sinetab <= "11111100";           -- 252
         WHEN 230 => sinetab <= "11111100";           -- 252
         WHEN 231 => sinetab <= "11111100";           -- 252
         WHEN 232 => sinetab <= "11111100";           -- 252
         WHEN 233 => sinetab <= "11111100";           -- 252
         WHEN 234 => sinetab <= "11111101";           -- 253
         WHEN 235 => sinetab <= "11111101";           -- 253
         WHEN 236 => sinetab <= "11111101";           -- 253
         WHEN 237 => sinetab <= "11111101";           -- 253
         WHEN 238 => sinetab <= "11111101";           -- 253
         WHEN 239 => sinetab <= "11111110";           -- 254
         WHEN 240 => sinetab <= "11111110";           -- 254
         WHEN 241 => sinetab <= "11111110";           -- 254
         WHEN 242 => sinetab <= "11111110";           -- 254
         WHEN 243 => sinetab <= "11111110";           -- 254
         WHEN 244 => sinetab <= "11111110";           -- 254
         WHEN 245 => sinetab <= "11111110";           -- 254
         WHEN 246 => sinetab <= "11111111";           -- 255
         WHEN 247 => sinetab <= "11111111";           -- 255
         WHEN 248 => sinetab <= "11111111";           -- 255
         WHEN 249 => sinetab <= "11111111";           -- 255
         WHEN 250 => sinetab <= "11111111";           -- 255
         WHEN 251 => sinetab <= "11111111";           -- 255
         WHEN 252 => sinetab <= "11111111";           -- 255
         WHEN 253 => sinetab <= "11111111";           -- 255
         WHEN 254 => sinetab <= "11111111";           -- 255
         WHEN 255 => sinetab <= "11111111";           -- 255
       END CASE;
   END PROCESS sine_table;

   --------------------------------------------------------------------------
   -- SINE LATCH PROCESS
   --------------------------------------------------------------------------
   -- The sine output is latched on falling edge of sin_le signal.
   --------------------------------------------------------------------------
   sine_latch: PROCESS (reset_int, sin_le, sinetab)
   BEGIN
      IF (reset_int = '0') THEN                       -- if reset is active
         sine <= (OTHERS => '0');                     -- sine output is zero
      ELSIF falling_edge(sin_le) THEN                 -- on sine latch low edge
          sine <= sinetab;                             -- sine output latched
      END IF;
   END PROCESS sine_latch;

   --------------------------------------------------------------------------
   -- COSINE LATCH PROCESS
   --------------------------------------------------------------------------
   -- The cosine output is latched on falling edge of cos_le signal.
   --------------------------------------------------------------------------
   cosine_latch: PROCESS (reset_int, cos_le, sinetab)
   BEGIN
      IF (reset_int = '0') THEN                       -- if reset is active
         cosine <= (OTHERS => '1');                   -- cosine output is max
      ELSIF falling_edge(cos_le) THEN                 -- on cosine latch low edge
         cosine <= sinetab;                           -- cosine output latched
      END IF;
   END PROCESS cosine_latch;

   --------------------------------------------------------------------------
   -- RESOLUTION SWITCH SETTINGS PROCESS
   --------------------------------------------------------------------------
   -- This selects the table step size for each switch settings.
   --------------------------------------------------------------------------
   res_select: PROCESS (last_sw)
   BEGIN
      CASE last_sw IS
         WHEN "0010" => stepsize <= 1;                -- 250 microsteps per step
         WHEN "0011" => stepsize <= 2;                -- 125 microsteps per step
         WHEN "0100" => stepsize <= 5;                -- 50 microsteps per step
         WHEN "0101" => stepsize <= 10;               -- 25 microsteps per step
         WHEN "0110" => stepsize <= 25;               -- 10 microsteps per step
         WHEN "0111" => stepsize <= 50;               -- 5 microsteps per step
         WHEN "1000" => stepsize <= 1;                -- 256 microsteps per step
         WHEN "1001" => stepsize <= 2;                -- 128 microsteps per step
         WHEN "1010" => stepsize <= 4;                -- 64 microsteps per step
         WHEN "1011" => stepsize <= 8;                -- 32 microsteps per step
         WHEN "1100" => stepsize <= 16;               -- 16 microsteps per step
         WHEN "1101" => stepsize <= 32;               -- 8 microsteps per step
         WHEN "1110" => stepsize <= 64;               -- 4 microsteps per step
         WHEN "1111" => stepsize <= 128;              -- 2 microsteps per step
         WHEN OTHERS => stepsize <= 16;               -- good coding practice
      END CASE;
   END PROCESS res_select;

END behaviour;
