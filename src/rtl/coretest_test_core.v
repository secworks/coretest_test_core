//======================================================================
//
// coretest_test_core.v
// --------------------
// Top level wrapper that creates the Cryptech coretest system.
// The wrapper contains instances of external interface, coretest
// and the core to be tested. And if more than one core is
// present the wrapper also includes address and data muxes.
//
// Author: Joachim Strombergson
// Copyright (c) 2014  Secworks Sweden AB
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or 
// without modification, are permitted provided that the following 
// conditions are met: 
// 
// 1. Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer. 
// 
// 2. Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the 
//    distribution. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module coretest_test_core(
                          input wire           clk,
                          input wire           reset_n
 
                          // External interface.
                          input wire          rxd,
                          output wire         txd,

                          output wire [7 : 0]  debug
                         );

  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------

  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  // uart connections
  wire          uart_rxd_syn;
  wire [7 : 0]  uart_rxd_data;
  wire          uart_rxd_ack;
  wire          uart_txd_syn;
  wire [7 : 0]  uart_txd_data;
  wire          uart_txd_ack;
  wire          uart_cs;
  wire          uart_we;
  wire [3 : 0]  uart_address;
  wire [31 : 0] uart_write_data;
  wire [31 : 0] uart_read_data;
  wire          uart_error;

  
  //----------------------------------------------------------------
  // Concurrent assignment.
  //----------------------------------------------------------------

  
  //----------------------------------------------------------------
  // Core instantiations.
  //----------------------------------------------------------------
  coretest coretest(
                    .clk(clk),
                    .reset_n(reset_n),
                         
                    .rx_syn(uart_rx_syn),
                    .rx_data(uart_rx_data),
                    .rx_ack(uart_rx_ack),
                    
                    .tx_syn(uart_tx_syn),
                    .tx_data(uart_tx_data),
                    .tx_ack(uart_tx_ack),
                    
                    // Interface to the core being tested.
                    .core_reset(coretest_core_reset),
                    .core_cs(coretest_core_cs),
                    .core_we(coretest_core_we),
                    .core_address(coretest_core_address),
                    .core_write_data(coretest_core_write_data),
                    .core_read_data(coretest_core_read_data),
                    .core_error(coretest_core_error)
                   );


  uart uart(
            .clk(clk),
            .reset_n(reset_n),
            
            // External interface.
            .rxd(rxd),
            .txd(txd),

            // Internal receive interface.
            .rxd_syn(),
            .rxd_data(),
            .rxd_ack(),

            // Internal transmit interface.
            .txd_syn(),
            .txd_data(),
            .txd_ack(),
            
            // API interface.
            .cs(),
            .we(),
            .address(),
            .write_data(),
            .read_data(),
            .error(),

            // Debug output.
            .debug()
           );

  
  test_core core(
                 .clk(),
                 .reset_n(),
                 
                 .cs(),
                 .we(),
                 .address(),
                 .write_data(),
                 .read_data(),
                 .error(),
                 
                 .debug()
                );

  
  //----------------------------------------------------------------
  // address_mux
  //
  // Combinational data mux that handles addressing between
  // cores using the 32-bit memory like interface.
  //----------------------------------------------------------------
  always @*
    begin : address_mux
      // Default assignments.
      
    end // address_mux
  
endmodule // coretest_test_core
