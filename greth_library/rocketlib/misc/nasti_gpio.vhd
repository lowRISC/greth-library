-----------------------------------------------------------------------------
--! @file
--! @copyright Copyright 2015 GNSS Sensor Ltd. All right reserved.
--! @author    Sergey Khabarov - sergeykhbr@gmail.com
--! @brief     Controller of the GPIOs with the AMBA AXI4 interface.
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
library commonlib;
use commonlib.types_common.all;
--! AMBA system bus specific library.
library ambalib;
--! AXI4 configuration constants.
use ambalib.types_amba4.all;


entity nasti_gpio is
  generic (
    xindex   : integer := 0;
    xaddr    : integer := 0;
    xmask    : integer := 16#fffff#
  );
  port (
    clk  : in std_logic;
    nrst : in std_logic;
    cfg  : out nasti_slave_config_type;
    i    : in  nasti_slave_in_type;
    o    : out nasti_slave_out_type;
    i_glip : in std_logic_vector(31 downto 0);
    o_glip : out std_logic_vector(31 downto 0)
  );
end; 
 
architecture arch_nasti_gpio of nasti_gpio is

  constant xconfig : nasti_slave_config_type := (
     xindex => xindex,
     xaddr => conv_std_logic_vector(xaddr, CFG_NASTI_CFG_ADDR_BITS),
     xmask => conv_std_logic_vector(xmask, CFG_NASTI_CFG_ADDR_BITS),
     vid => VENDOR_GNSSSENSOR,
     did => GNSSSENSOR_GPIO,
     descrtype => PNP_CFG_TYPE_SLAVE,
     descrsize => PNP_CFG_SLAVE_DESCR_BYTES
  );

  type local_addr_array_type is array (0 to CFG_WORDS_ON_BUS-1) 
       of integer;

  type bank_type is record
    o_glip : std_logic_vector(31 downto 0);
    i_glip : std_logic_vector(31 downto 0);
  end record;
  
  type registers is record
    bank_axi : nasti_slave_bank_type;
    bank0 : bank_type;
  end record;

  constant RESET_VALUE : registers := (
        NASTI_SLAVE_BANK_RESET,
        ((others => '0'), (others => '0'))
  );

  signal r, rin : registers;
  signal r_istb, r_ostb, r_istb2, r_ostb2, r_istbin, r_ostbin : std_logic;

begin

  comblogic : process(i, i_glip, r, nrst)
    variable istb, ostb : std_logic;
    variable v : registers;
    variable raddr_reg : local_addr_array_type;
    variable waddr_reg : local_addr_array_type;
    variable rdata : std_logic_vector(CFG_NASTI_DATA_BITS-1 downto 0);
    variable tmp : std_logic_vector(31 downto 0);
    variable wstrb : std_logic_vector(CFG_NASTI_DATA_BYTES-1 downto 0);
  begin

    v := r;
    istb := '0';
    ostb := '0';
    
    procedureAxi4(i, xconfig, r.bank_axi, v.bank_axi);

    for n in 0 to CFG_WORDS_ON_BUS-1 loop
      raddr_reg(n) := conv_integer(r.bank_axi.raddr(n)(11 downto 2));
      tmp := (others => '0');

      case raddr_reg(n) is
        when 0 => tmp := r.bank0.o_glip;
        when 1 => tmp := r.bank0.i_glip; istb := '1';
        when 2 => tmp := r.bank0.i_glip;
        when others =>
      end case;
      rdata(8*CFG_ALIGN_BYTES*(n+1)-1 downto 8*CFG_ALIGN_BYTES*n) := tmp;
    end loop;


    if i.w_valid = '1' and 
       r.bank_axi.wstate = wtrans and 
       r.bank_axi.wresp = NASTI_RESP_OKAY then

       wstrb := i.w_strb;
       for n in 0 to CFG_WORDS_ON_BUS-1 loop
         waddr_reg(n) := conv_integer(r.bank_axi.waddr(n)(11 downto 2));
         tmp := i.w_data(32*(n+1)-1 downto 32*n);

         if conv_integer(wstrb(CFG_ALIGN_BYTES*(n+1)-1 downto CFG_ALIGN_BYTES*n)) /= 0 then
           v.bank0.o_glip := tmp; ostb := '1' ;
         end if;
       end loop;
    end if;

    o <= functionAxi4Output(r.bank_axi, rdata);

    v.bank0.i_glip := i_glip;
    
    if nrst = '0' then
        v := RESET_VALUE;
    end if;
  
    rin <= v;
    r_istbin <= istb;
    r_ostbin <= ostb;
  end process;

  cfg  <= xconfig;
  
  o_glip <= (r_ostb and not r_ostb2) & (r_istb and not r_istb2) & r.bank0.o_glip(29 downto 0);

  -- registers:
  regs : process(clk)
  begin 
     if rising_edge(clk) then 
        r <= rin;
        r_istb <= r_istbin;
        r_ostb <= r_ostbin;
        r_istb2 <= r_istb;
        r_ostb2 <= r_ostb;
        
     end if; 
  end process;

end;
