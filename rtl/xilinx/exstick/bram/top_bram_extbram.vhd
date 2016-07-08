--
-- Copyright (c) 2015 Marko Zec, University of Zagreb
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
-- FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
-- DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
-- LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
-- OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--
-- $Id$
--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_std.ALL;

library unisim;
use unisim.vcomponents.all;

use work.f32c_pack.all;


entity glue is
  generic (
    -- ISA
    C_arch: integer := ARCH_MI32;
    C_debug: boolean := false;
    -- Main clock: N * 10 MHz
    C_clk_freq: integer := 50;

    -- SoC configuration options
    C_mem_size  : integer := 16;
    C_sio       : integer := 1;
    C_spi       : integer := 2;
    C_gpio      : integer := 32;
    C_simple_io : boolean := true
    );
  port (
    clk_25m            : in  std_logic;
    rs232_dce_txd      : out std_logic;
    rs232_dce_rxd      : in  std_logic;
    seg                : out std_logic_vector(7 downto 0);  -- 7-segment display
    an                 : out std_logic_vector(3 downto 0);  -- 7-segment display
    led                : out std_logic_vector(7 downto 0);
    btn_center         : in  std_logic;
    btn_south          : in  std_logic;
    btn_north          : in  std_logic;
    btn_east           : in  std_logic;
    btn_west           : in  std_logic;
    sw                 : in  std_logic_vector(7 downto 0);

    -- GMII PHY : 2.5V
    ETH_RESET_N : out   std_logic;      -- このchipにおけるシステムリセットを出す
    -- MAC I/F
    ETH_COL     : inout std_logic;      -- CollisionDetect/MAC_FREQ兼用ピン。外部で2kPUされている。＝＞MAC_CLK出力を125MHzにしている様。
    ETH_CRS     : inout std_logic;      -- CarrierSence/RGMII_SEL0兼用ピン。外部で2kPDされている。
    ETH_TX_CLK  : in    std_logic;      -- TXCLK/RGMII_SEL1兼用ピン。外部で2kPDされている。＝＞CRSと合わせてGMIIモードにしている。
    -- TX out
    ETH_TX_D    : out   std_logic_vector(3 downto 0);      -- 送信データ
    ETH_TX_EN   : out   std_logic;
    -- RX in
    ETH_RX_D    : inout std_logic_vector(3 downto 0);      -- 受信データ
    ETH_RX_CLK  : in    std_logic;      -- 10M=>2.5MHz, 100M=>25MHz, 1G=>125MHz のクロックが来る
    ETH_RX_DV   : inout std_logic;
    ETH_RX_ER   : inout std_logic;
    -- Management I/F
    ETH_MDC     : out   std_logic;      -- クロック出力。max.2.5MHz。
    ETH_MDIO    : inout std_logic;      -- コントロールデータ。
    ETH_PDW     : out   std_logic
    
    );
end glue;

architecture Behavioral of glue is
  signal clk, rs232_break: std_logic;
  signal ja, jb, jc, jd : std_logic_vector(7 downto 0);  -- PMODs dummy

  component glue_bram_extbram
    generic (
      C_clk_freq: integer;

      -- ISA options
      C_arch: integer := ARCH_MI32;
      C_big_endian: boolean := false;
      C_mult_enable: boolean := true;
      C_branch_likely: boolean := true;
      C_sign_extend: boolean := true;
      C_ll_sc: boolean := false;
      C_PC_mask: std_logic_vector(31 downto 0) := x"0001ffff"; -- 128 K
      C_exceptions: boolean := true;

      -- COP0 options
      C_cop0_count: boolean := true;
      C_cop0_compare: boolean := true;
      C_cop0_config: boolean := true;

      -- CPU core configuration options
      C_branch_prediction: boolean := true;
      C_full_shifter: boolean := true;
      C_result_forwarding: boolean := true;
      C_load_aligner: boolean := true;

      -- Negatively influences timing closure, hence disabled
      C_movn_movz: boolean := false;

      -- CPU debugging
      C_debug: boolean := false;

      -- SoC configuration options
      C_bram_size: integer := 16;	-- in KBytes
      C_boot_spi: boolean := false;
      C_sio: integer := 1;
      C_sio_init_baudrate: integer := 115200;
      C_sio_fixed_baudrate: boolean := false;
      C_sio_break_detect: boolean := true;
      C_spi: integer := 0;
      C_spi_turbo_mode: std_logic_vector := "0000";
      C_spi_fixed_speed: std_logic_vector := "1111";
      C_simple_in: integer range 0 to 128 := 32;
      C_simple_out: integer range 0 to 128 := 32;
      C_gpio: integer range 0 to 128 := 32;
      C_timer: boolean := true
      );
    port (
      clk: in std_logic;
      sio_rxd: in std_logic_vector(C_sio - 1 downto 0);
      sio_txd, sio_break: out std_logic_vector(C_sio - 1 downto 0);
      spi_sck, spi_ss, spi_mosi: out std_logic_vector(C_spi - 1 downto 0);
      spi_miso: in std_logic_vector(C_spi - 1 downto 0);
      simple_in: in std_logic_vector(31 downto 0);
      simple_out: out std_logic_vector(31 downto 0);
      gpio: inout std_logic_vector(127 downto 0);
      extbram_addr : out std_logic_vector(27 downto 0);
      extbram_write : out std_logic;
      extbram_byte_sel : out std_logic_vector(3 downto 0);
      extbram_data_in : in std_logic_vector(31 downto 0);
      extbram_data_out : out std_logic_vector(31 downto 0)
      );
  end component glue_bram_extbram;

  signal extbram_addr        : std_logic_vector(27 downto 0);
  signal extbram_write       : std_logic;
  signal extbram_byte_sel    : std_logic_vector(3 downto 0);
  signal extbram_data_in     : std_logic_vector(31 downto 0);
  signal extbram_data_out    : std_logic_vector(31 downto 0);
  signal extbram_data_in_0   : std_logic_vector(31 downto 0);
  signal extbram_data_in_1   : std_logic_vector(31 downto 0);
  signal extbram_data_in_sel : std_logic;

  attribute mark_debug : string;
  attribute keep       : string;
  attribute S          : string;

  attribute keep of extbram_addr        : signal is "true";
  attribute keep of extbram_write       : signal is "true";
  attribute keep of extbram_byte_sel    : signal is "true";
  attribute keep of extbram_data_in     : signal is "true";
  attribute keep of extbram_data_out    : signal is "true";

  attribute mark_debug of extbram_addr        : signal is "true";
  attribute mark_debug of extbram_write       : signal is "true";
  attribute mark_debug of extbram_byte_sel    : signal is "true";
  attribute mark_debug of extbram_data_in     : signal is "true";
  attribute mark_debug of extbram_data_out    : signal is "true";

  attribute S of extbram_addr        : signal is "true";
  attribute S of extbram_write       : signal is "true";
  attribute S of extbram_byte_sel    : signal is "true";
  attribute S of extbram_data_in     : signal is "true";
  attribute S of extbram_data_out    : signal is "true";

  COMPONENT dualport_blockram
    PORT (
      clka : IN STD_LOGIC;
      wea : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
      addra : IN STD_LOGIC_VECTOR(8 DOWNTO 0);
      dina : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
      douta : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      clkb : IN STD_LOGIC;
      web : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
      addrb : IN STD_LOGIC_VECTOR(8 DOWNTO 0);
      dinb : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
      doutb : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
      );
  END COMPONENT;

  signal send_buf_we   : std_logic := '0';
  signal send_buf_addr : unsigned(8 downto 0);
  signal send_buf_din  : std_logic_vector(31 downto 0);
  signal send_buf_dout : std_logic_vector(31 downto 0);

  signal recv_buf_we   : std_logic := '0';
  signal recv_buf_addr : unsigned(8 downto 0);
  signal recv_buf_din  : std_logic_vector(31 downto 0);
  signal recv_buf_dout : std_logic_vector(31 downto 0);
  
  component e7udpip100M_exstick
    port(
      -- GMII PHY : 2.5V
      EPHY_RST_N    : out   std_logic ;  -- このchipにおけるシステムリセットを出す
      EPHY_MAC_CLK  : in    std_logic ;  -- PHYからの25MHzCLK入力
      -- MAC I/F
      EPHY_COL      : inout std_logic ;  -- CollisionDetect/MAC_FREQ兼用ピン。外部で2kPUされている。＝＞MAC_CLK出力を125MHzにしている様。
      EPHY_CRS      : inout std_logic ;  -- CarrierSence/RGMII_SEL0兼用ピン。外部で2kPDされている。
      EPHY_TXCLK    : in    std_logic ;  -- TXCLK/RGMII_SEL1兼用ピン。外部で2kPDされている。＝＞CRSと合わせてGMIIモードにしている。
      -- TX out
      EPHY_TD       : out   std_logic_vector( 3 downto 0 ) ;  -- 送信データ
      EPHY_TXEN     : out   std_logic ;
      EPHY_TXER     : out   std_logic ;
      -- RX in
      EPHY_RD       : inout std_logic_vector( 3 downto 0 ) ;  -- 受信データ
      EPHY_RXCLK    : in    std_logic ;  -- 10M=>2.5MHz, 100M=>25MHz, 1G=>125MHz のクロックが来る
      EPHY_RXDV     : inout std_logic ;
      EPHY_RXER     : inout std_logic ;
      -- Management I/F
      EPHY_MDC      : out   std_logic ;  -- クロック出力。max.2.5MHz。
      EPHY_MDIO     : inout std_logic ;  -- コントロールデータ。
      EPHY_INT_N    : in    std_logic ;  -- インタラプト。

      -- Asynchronous Reset
      Reset_n        : in  std_logic;

      -- UPL interface
      pUPLGlobalClk : in  std_logic;
      -- UDP tx input
      pUdp0Send_Data       : in  std_logic_vector( 31 downto 0 );
      pUdp0Send_Request    : in  std_logic;
      pUdp0Send_Ack        : out std_logic;
      pUdp0Send_Enable     : in  std_logic;

      pUdp1Send_Data       : in  std_logic_vector( 31 downto 0 );
      pUdp1Send_Request    : in  std_logic;
      pUdp1Send_Ack        : out std_logic;
      pUdp1Send_Enable     : in  std_logic;

      -- UDP rx output
      pUdp0Receive_Data       : out std_logic_vector( 31 downto 0 );
      pUdp0Receive_Request    : out std_logic;
      pUdp0Receive_Ack        : in  std_logic;
      pUdp0Receive_Enable     : out std_logic;

      pUdp1Receive_Data       : out std_logic_vector( 31 downto 0 );
      pUdp1Receive_Request    : out std_logic;
      pUdp1Receive_Ack        : in  std_logic;
      pUdp1Receive_Enable     : out std_logic;

      -- Setup
      pMyIpAddr       : in std_logic_vector( 31 downto 0 );
      pMyMacAddr      : in std_logic_vector( 47 downto 0 );
      pMyNetmask      : in std_logic_vector( 31 downto 0 );
      pDefaultGateway : in std_logic_vector( 31 downto 0 );
      pTargetIPAddr   : in std_logic_vector( 31 downto 0 );
      pMyUdpPort0     : in std_logic_vector( 15 downto 0 );
      pMyUdpPort1     : in std_logic_vector( 15 downto 0 );

      -- Status
      pStatus_RxByteCount             : out std_logic_vector( 31 downto 0 );
      pStatus_RxPacketCount           : out std_logic_vector( 31 downto 0 );
      pStatus_RxErrorPacketCount      : out std_logic_vector( 15 downto 0 );
      pStatus_RxDropPacketCount       : out std_logic_vector( 15 downto 0 );
      pStatus_RxARPRequestPacketCount : out std_logic_vector( 15 downto 0 );
      pStatus_RxARPReplyPacketCount   : out std_logic_vector( 15 downto 0 );
      pStatus_RxICMPPacketCount       : out std_logic_vector( 15 downto 0 );
      pStatus_RxUDP0PacketCount       : out std_logic_vector( 15 downto 0 );
      pStatus_RxUDP1PacketCount       : out std_logic_vector( 15 downto 0 );
      pStatus_RxIPErrorPacketCount    : out std_logic_vector( 15 downto 0 );
      pStatus_RxUDPErrorPacketCount   : out std_logic_vector( 15 downto 0 );

      pStatus_TxByteCount             : out std_logic_vector( 31 downto 0 );
      pStatus_TxPacketCount           : out std_logic_vector( 31 downto 0 );
      pStatus_TxARPRequestPacketCount : out std_logic_vector( 15 downto 0 );
      pStatus_TxARPReplyPacketCount   : out std_logic_vector( 15 downto 0 );
      pStatus_TxICMPReplyPacketCount  : out std_logic_vector( 15 downto 0 );
      pStatus_TxUDP0PacketCount       : out std_logic_vector( 15 downto 0 );
      pStatus_TxUDP1PacketCount       : out std_logic_vector( 15 downto 0 );
      pStatus_TxMulticastPacketCount  : out std_logic_vector( 15 downto 0 );

      pStatus_phy_link_100M       : out std_logic;
      pStatus_phy_link_fullduplex : out std_logic;
      pStatus_phy_link_up         : out std_logic;
      
      pdebug : out std_logic_vector(63 downto 0)
      );
  end component;
  
  signal reset_counter : unsigned(7 downto 0) := (others => '0');
  signal reset_n : std_logic := '0';
  signal pdebug : std_logic_vector(63 downto 0);

  signal MyIpAddr     : std_logic_vector( 31 downto 0 ) := X"0a000010";
  signal MyNetmask    : std_logic_vector( 31 downto 0 ) := X"ffff0000";
  signal DefaultGateway : std_logic_vector( 31 downto 0 ) := X"0a0000fe";
  signal MyMacAddr    : std_logic_vector( 47 downto 0 ) := X"001b1aff0001";
  signal MyUdpPort0   : std_logic_vector( 15 downto 0 ) := X"4000";
  signal MyUdpPort1   : std_logic_vector( 15 downto 0 ) := X"4001";
  signal serverIpAddr : std_logic_vector( 31 downto 0 ) := X"0a000003";
  signal multicastIpAddr : std_logic_vector( 31 downto 0 ) := X"e0000003";

  signal Status_RxByteCount             : std_logic_vector( 31 downto 0 );
  signal Status_RxPacketCount           : std_logic_vector( 31 downto 0 );
  signal Status_RxErrorPacketCount      : std_logic_vector( 15 downto 0 );
  signal Status_RxDropPacketCount       : std_logic_vector( 15 downto 0 );
  signal Status_RxARPRequestPacketCount : std_logic_vector( 15 downto 0 );
  signal Status_RxARPReplyPacketCount   : std_logic_vector( 15 downto 0 );
  signal Status_RxICMPPacketCount       : std_logic_vector( 15 downto 0 );
  signal Status_RxUDP0PacketCount       : std_logic_vector( 15 downto 0 );
  signal Status_RxUDP1PacketCount       : std_logic_vector( 15 downto 0 );
  signal Status_RxIPErrorPacketCount    : std_logic_vector( 15 downto 0 );
  signal Status_RxUDPErrorPacketCount   : std_logic_vector( 15 downto 0 );

  signal Status_TxByteCount             : std_logic_vector( 31 downto 0 );
  signal Status_TxPacketCount           : std_logic_vector( 31 downto 0 );
  signal Status_TxARPRequestPacketCount : std_logic_vector( 15 downto 0 );
  signal Status_TxARPReplyPacketCount   : std_logic_vector( 15 downto 0 );
  signal Status_TxICMPReplyPacketCount  : std_logic_vector( 15 downto 0 );
  signal Status_TxUDP0PacketCount       : std_logic_vector( 15 downto 0 );
  signal Status_TxUDP1PacketCount       : std_logic_vector( 15 downto 0 );
  signal Status_TxMulticastPacketCount  : std_logic_vector( 15 downto 0 );

  signal UDP0Send_Data : std_logic_vector(31 downto 0);
  signal UDP0Send_Enable, UDP0Send_Request, UDP0Send_Ack : std_logic;
  signal UDP1Send_Data : std_logic_vector(31 downto 0);
  signal UDP1Send_Enable, UDP1Send_Request, UDP1Send_Ack : std_logic;
  signal UDP0Receive_Data : std_logic_vector(31 downto 0);
  signal UDP0Receive_Enable, UDP0Receive_Request, UDP0Receive_Ack : std_logic;
  signal UDP1Receive_Data : std_logic_vector(31 downto 0);
  signal UDP1Receive_Enable, UDP1Receive_Request, UDP1Receive_Ack : std_logic;

  signal Status_phy_link_100M       : std_logic;
  signal Status_phy_link_fullduplex : std_logic;
  signal Status_phy_link_up         : std_logic;

  type uplSendStateType is (SEND_IDLE, SEND_BODY, SEND_BRAM_WAIT, SEND_ACK_WAIT, SEND_CLEANUP);
  signal uplSendState : uplSendStateType := SEND_IDLE;

  signal send_kick_flag_d : std_logic := '0';
  signal send_length : unsigned(15 downto 0) := (others => '0');

  type uplRecvStateType is (RECV_IDLE, RECV_BODY, RECV_READY, RECV_READWAIT);
  signal uplRecvState : uplRecvStateType := RECV_IDLE;
  signal recv_length : unsigned(15 downto 0) := (others => '0');

begin
  
  ETH_PDW <= '1';
  
  -- clock synthesizer: Xilinx Spartan-6 specific
  clkgen: entity work.clkgen
    generic map(
      C_clk_freq => C_clk_freq
      )
    port map(
      clk_25m => clk_25m, clk => clk
      );

  -- reset hard-block: Xilinx Spartan-6 specific
  reset: startup_spartan6
    port map (
      clk => clk, gsr => rs232_break, gts => rs232_break,
      keyclearb => '0'
      );

  -- generic BRAM glue
  glue_bram: entity work.glue_bram_extbram
    generic map (
      C_clk_freq => C_clk_freq,
      C_arch => C_arch,
      C_bram_size => C_mem_size,
      C_gpio => C_gpio,
      C_sio => C_sio,
      C_spi => C_spi,
      C_debug => C_debug
      )
    port map (
      clk => clk,
      sio_txd(0) => rs232_dce_txd, sio_rxd(0) => rs232_dce_rxd,
      sio_break(0) => rs232_break,
      spi_sck(0)  => open,  spi_sck(1)  => open,
      spi_ss(0)   => open,  spi_ss(1)   => open,
      spi_mosi(0) => open,  spi_mosi(1) => open,
      spi_miso(0) => '-',   spi_miso(1) => '-',
      gpio(7 downto 0) => ja(7 downto 0),
      gpio(15 downto 8) => jb(7 downto 0),
      gpio(23 downto 16) => jc(7 downto 0),
      gpio(31 downto 24) => jd(7 downto 0),
      gpio(127 downto 32) => open,
      simple_out(7 downto 0) => led(7 downto 0),
      simple_out(15 downto 8) => seg(7 downto 0),
      simple_out(19 downto 16) => an(3 downto 0),
      simple_out(31 downto 20) => open,
      simple_in(0) => btn_west, simple_in(1) => btn_east,
      simple_in(2) => btn_north, simple_in(3) => btn_south,
      simple_in(4) => btn_center,
      simple_in(12 downto 5) => sw(7 downto 0),
      simple_in(31 downto 13) => open,
      
      extbram_addr        => extbram_addr,
      extbram_write       => extbram_write,
      extbram_byte_sel    => extbram_byte_sel,
      extbram_data_in     => extbram_data_in,
      extbram_data_out    => extbram_data_out
      
      );

  SEND_BUF : dualport_blockram
    PORT map(
      clka   => not clk,
      wea(0) => extbram_write and (not extbram_addr(9)),
      addra  => extbram_addr(8 downto 0),
      dina   => extbram_data_out,
      douta  => extbram_data_in_0,
      clkb   => clk,
      web(0) => send_buf_we,
      addrb  => std_logic_vector(send_buf_addr),
      dinb   => send_buf_din,
      doutb  => send_buf_dout
      );

  RECV_BUF : dualport_blockram
    PORT map(
      clka   => not clk,
      wea(0) => extbram_write and extbram_addr(9),
      addra  => extbram_addr(8 downto 0),
      dina   => extbram_data_out,
      douta  => extbram_data_in_1,
      clkb   => clk,
      web(0) => recv_buf_we,
      addrb  => std_logic_vector(recv_buf_addr),
      dinb   => recv_buf_din,
      doutb  => recv_buf_dout
      );

  extbram_data_in <= extbram_data_in_0 when extbram_data_in_sel = '0' else extbram_data_in_1;
  
  process(clk)
  begin
    if falling_edge(clk) then
      if extbram_addr(9) = '0' then
        extbram_data_in_sel <= '0';
      else
        extbram_data_in_sel <= '1';
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      case (uplSendState) is
        -- wait for send request from SW
        when SEND_IDLE =>
          send_buf_addr    <= (others => '0');
          send_buf_we      <= '0';
          UDP0Send_Enable  <= '0';
          send_kick_flag_d <= send_buf_dout(31);
          if send_kick_flag_d = '0' and
             send_buf_dout(31) = '1' and
             unsigned(send_buf_dout(15 downto 0)) > 0 then
            send_length      <= unsigned(send_buf_dout(15 downto 0));
            uplSendState     <= SEND_ACK_WAIT;
            UDP0Send_Request <= '1';
          else
            UDP0Send_Request <= '0';
          end if;

        -- wait for ACK from e7UDPIP IP-core
        when SEND_ACK_WAIT =>
          send_kick_flag_d <= '1';
          if UDP0Send_Ack = '1' then
            UDP0Send_Request <= '0';
            send_buf_addr    <= send_buf_addr + 1;
            uplSendState     <= SEND_BRAM_WAIT;
          else
            UDP0Send_Request <= '1';
          end if;
          
        -- 1 clock wait for BRAM latency
        when SEND_BRAM_WAIT =>          -- bram wait
          send_buf_addr <= send_buf_addr + 1;
          uplSendState  <= SEND_BODY;
          
        -- send data to e7UDPIP IP-core
        when SEND_BODY =>
          send_buf_addr   <= send_buf_addr + 1;
          UDP0Send_Data   <= send_buf_dout;
          UDP0Send_Enable <= '1';
          send_length     <= send_length - 1;
          if send_length = 1 then     -- last word
            uplSendState  <= SEND_CLEANUP;
            send_buf_addr <= (others => '0');
          else
            send_buf_addr <= send_buf_addr + 1;
          end if;

        -- cleanup
        when SEND_CLEANUP =>
          uplSendState    <= SEND_IDLE;
          UDP0Send_Enable <= '0';
          send_buf_we     <= '1';
          send_buf_addr   <= (others => '0');
          send_buf_din    <= (others => '0');  -- clear the flag and length

        when others =>
          uplSendState <= SEND_CLEANUP;
          
      end case;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      case (uplRecvState) is
        -- wait for receiving an UDP packet
        when RECV_IDLE =>
          recv_buf_addr <= (others => '0');
          recv_buf_we   <= '0';
          recv_buf_din  <= (others => '0');
          recv_length   <= (others => '0');
          if UDP0Receive_Request = '1' then
            uplRecvState    <= RECV_BODY;
            UDP0Receive_Ack <= '1';
          else
            UDP0Receive_Ack <= '0';
          end if;

        -- receive data from UDP core
        when RECV_BODY =>
          if UDP0Receive_Request = '0' then
            UDP0Receive_Ack <= '0';
          end if;
          if UDP0Receive_Enable = '1' then
            recv_buf_we   <= '1';
            recv_buf_addr <= recv_length(8 downto 0) + 1;  -- to reserve addr 0 for cmd/sts
            recv_buf_din  <= UDP0Receive_Data;
            recv_length   <= recv_length + 1;
          elsif UDP0Receive_Enable = '0' and recv_length > 0 then
            -- should receive a word, at least
            uplRecvState  <= RECV_READY;
            recv_buf_we   <= '0';
            recv_buf_addr <= (others => '0');
          end if;
          
        -- update status
        when RECV_READY =>          -- bram wait
          recv_buf_addr    <= (others => '0');
          recv_buf_we      <= '1';
          recv_buf_din(31) <= '1';
          recv_buf_din(15 downto 0) <= std_logic_vector(recv_length);
          uplRecvState <= RECV_READWAIT;
          
        -- wait for clearing status by SW
        when RECV_READWAIT =>
          recv_buf_we   <= '0';
          recv_buf_addr <= (others => '0');
          if unsigned(recv_buf_dout) = 0 then
            uplRecvState    <= RECV_IDLE;  -- to receive a next packet
          end if;

        when others =>
          uplRecvState<= RECV_IDLE;
          
      end case;
    end if;
  end process;
  
  process(clk_25m)
  begin
    if rising_edge(clk_25m) then
      if reset_counter < 100 then
        reset_counter <= reset_counter + 1;
        reset_n <= '0';
      else
        reset_n <= '1';
      end if;
    end if;
  end process;

  u_e7udpip0 : e7udpip100M_exstick
    port map (
      -- GMII PHY : 2.5V
      EPHY_RST_N    => ETH_RESET_N,
      EPHY_MAC_CLK  => clk_25m, -- USER_CLOCK,
      -- MAC I/F
      EPHY_COL      => ETH_COL,
      EPHY_CRS      => ETH_CRS,
      EPHY_TXCLK    => ETH_TX_CLK,
      -- TX out
      EPHY_TD       => ETH_TX_D,
      EPHY_TXEN     => ETH_TX_EN,
      EPHY_TXER     => open,
      -- RX in
      EPHY_RD       => ETH_RX_D,
      EPHY_RXCLK    => ETH_RX_CLK,
      EPHY_RXDV     => ETH_RX_DV,
      EPHY_RXER     => ETH_RX_ER,
      -- Management I/F
      EPHY_MDC      => ETH_MDC,
      EPHY_MDIO     => ETH_MDIO,
      EPHY_INT_N    => '1',

      -- Asynchronous Reset
      Reset_n       => reset_n,
      -- UPL interface
      pUPLGlobalClk => clk,
      -- UDP tx input
      pUdp0Send_Data       => UDP0Send_Data,
      pUdp0Send_Request    => UDP0Send_Request,
      pUdp0Send_Ack        => UDP0Send_Ack,
      pUdp0Send_Enable     => UDP0Send_Enable,

      pUdp1Send_Data       => UDP1Receive_Data,
      pUdp1Send_Request    => UDP1Receive_Request,
      pUdp1Send_Ack        => UDP1Receive_Ack,
      pUdp1Send_Enable     => UDP1Receive_Enable,

      -- UDP rx output
      pUdp0Receive_Data       => Udp0Receive_Data,
      pUdp0Receive_Request    => Udp0Receive_Request,
      pUdp0Receive_Ack        => Udp0Receive_Ack,
      pUdp0Receive_Enable     => Udp0Receive_Enable,

      pUdp1Receive_Data       => Udp1Receive_Data,
      pUdp1Receive_Request    => Udp1Receive_Request,
      pUdp1Receive_Ack        => Udp1Receive_Ack,
      pUdp1Receive_Enable     => Udp1Receive_Enable,

      -- Setup
      pMyIpAddr       => MyIpAddr,
      pMyMacAddr      => MyMacAddr,
      pMyNetmask      => MyNetmask,
      pDefaultGateway => DefaultGateway,
      pMyUdpPort0     => MyUdpPort0,
      pMyUdpPort1     => MyUdpPort1,
      pTargetIpAddr   => serveripaddr,

      -- Status
      pStatus_RxByteCount             => Status_RxByteCount,
      pStatus_RxPacketCount           => Status_RxPacketCount,
      pStatus_RxErrorPacketCount      => Status_RxErrorPacketCount,
      pStatus_RxDropPacketCount       => Status_RxDropPacketCount,
      pStatus_RxARPRequestPacketCount => Status_RxARPRequestPacketCount,
      pStatus_RxARPReplyPacketCount   => Status_RxARPReplyPacketCount,
      pStatus_RxICMPPacketCount       => Status_RxICMPPacketCount,
      pStatus_RxUDP0PacketCount       => Status_RxUDP0PacketCount,
      pStatus_RxUDP1PacketCount       => Status_RxUDP1PacketCount,
      pStatus_RxIPErrorPacketCount    => Status_RxIPErrorPacketCount,
      pStatus_RxUDPErrorPacketCount   => Status_RxUDPErrorPacketCount,

      pStatus_TxByteCount             => Status_TxByteCount,
      pStatus_TxPacketCount           => Status_TxPacketCount,
      pStatus_TxARPRequestPacketCount => Status_TxARPRequestPacketCount,
      pStatus_TxARPReplyPacketCount   => Status_TxARPReplyPacketCount,
      pStatus_TxICMPReplyPacketCount  => Status_TxICMPReplyPacketCount,
      pStatus_TxUDP0PacketCount       => Status_TxUDP0PacketCount,
      pStatus_TxUDP1PacketCount       => Status_TxUDP1PacketCount,
      pStatus_TxMulticastPacketCount  => Status_TxMulticastPacketCount,

      pStatus_phy_link_100M       => Status_phy_link_100M,
      pStatus_phy_link_fullduplex => Status_phy_link_fullduplex,
      pStatus_phy_link_up         => Status_phy_link_up,

      pdebug => pdebug
      );

end Behavioral;
