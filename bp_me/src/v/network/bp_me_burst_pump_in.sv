/**
 *
 * Name:
 *   bp_me_burst_pump_in.sv
 *
 * Description:
 *   Provides an FSM with control signals for an inbound BedRock Burst interface.
 *   This module buffers the inbound BedRock Burst channel and exposes it to the FSM.
 *
 */

`include "bp_common_defines.svh"
`include "bp_me_defines.svh"

module bp_me_burst_pump_in
 import bp_common_pkg::*;
 import bp_me_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   , parameter `BSG_INV_PARAM(stream_data_width_p)
   , parameter `BSG_INV_PARAM(block_width_p)
   // width of BedRock message payload
   , parameter `BSG_INV_PARAM(payload_width_p)

   // Bitmasks that specify which message types may have multiple beats on either
   // the msg input side or FSM output side.
   // Each mask is constructed as (1 << e_rd/wr_msg | 1 << e_uc_rd/wr_msg)
   // There are two cases:
   // 1. Message types that are set as part of fsm_stream_mask_p but not set in
   //    msg_stream_mask_p result in a 1:N conversion from msg->FSM ports.
   //    For example, in BlackParrot a read command for 64B to the
   //    cache arriving on the BedRock Bust input can be decomposed into a stream of
   //    8B reads on the FSM output port.
   // 2. Message types set in both will have N:N beats. Every beat on the input
   //    will produce a beat on the output. This is commonly used for all messages
   //    with data payloads.
   // Constructed as (1 << e_rd/wr_msg | 1 << e_uc_rd/wr_msg)
   , parameter msg_stream_mask_p = 0
   , parameter fsm_stream_mask_p = msg_stream_mask_p

   `declare_bp_bedrock_if_widths(paddr_width_p, payload_width_p, xce)

   , localparam block_offset_width_lp = `BSG_SAFE_CLOG2(block_width_p >> 3)
   , localparam stream_bytes_lp = stream_data_width_p >> 3
   , localparam stream_offset_width_lp = `BSG_SAFE_CLOG2(stream_bytes_lp)
   , localparam stream_words_lp = block_width_p / stream_data_width_p
   , localparam stream_cnt_width_lp = `BSG_SAFE_CLOG2(stream_words_lp)

   // number of messages that can be buffered
   , parameter header_els_p = 2
   , parameter data_els_p   = header_els_p * stream_words_lp
   )
  (input                                            clk_i
   , input                                          reset_i

   // Input BedRock Burst
   , input [xce_header_width_lp-1:0]                msg_header_i
   , input                                          msg_header_v_i
   , output logic                                   msg_header_ready_and_o
   , input                                          msg_has_data_i

   , input [stream_data_width_p-1:0]                msg_data_i
   , input                                          msg_data_v_i
   , output logic                                   msg_data_ready_and_o
   , input                                          msg_last_i

   // FSM consumer side
   , output logic [xce_header_width_lp-1:0]         fsm_header_o
   , output logic [stream_cnt_width_lp-1:0]         fsm_cnt_o 
   , output logic [paddr_width_p-1:0]               fsm_addr_o
   , output logic [stream_data_width_p-1:0]         fsm_data_o
   , output logic                                   fsm_v_o
   , input                                          fsm_yumi_i
   // FSM control signals
   // fsm_new is raised on first beat of every message
   , output logic                                   fsm_new_o
   // fsm_last is raised on last beat of every message
   , output logic                                   fsm_last_o
   );

  `declare_bp_bedrock_if(paddr_width_p, payload_width_p, lce_id_width_p, lce_assoc_p, xce);
  `bp_cast_i(bp_bedrock_xce_header_s, msg_header);
  `bp_cast_o(bp_bedrock_xce_header_s, fsm_header);

  enum logic [1:0]{e_ready, e_spray, e_burst} state_n, state_r;
  wire is_ready = (state_r == e_ready);
  wire is_spray = (state_r == e_spray);
  wire is_burst = (state_r == e_burst);

  bp_bedrock_xce_header_s msg_header_li;
  logic msg_header_v_li, msg_header_yumi_lo, msg_has_data_li;
  bsg_fifo_1r1w_small
   #(.width_p(1+xce_header_width_lp), .els_p(header_els_p))
   header_fifo
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.data_i({msg_has_data_i, msg_header_cast_i})
     ,.v_i(msg_header_v_i)
     ,.ready_o(msg_header_ready_and_o)

     ,.data_o({msg_has_data_li, msg_header_li})
     ,.v_o(msg_header_v_li)
     ,.yumi_i(msg_header_yumi_lo)
     );

  logic [stream_data_width_p-1:0] msg_data_li;
  logic msg_data_v_li, msg_data_yumi_lo, msg_last_li;
  bsg_fifo_1r1w_small
   #(.width_p(1+stream_data_width_p), .els_p(data_els_p))
   data_fifo
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.data_i({msg_last_i, msg_data_i})
     ,.v_i(msg_data_v_i)
     ,.ready_o(msg_data_ready_and_o)

     ,.data_o({msg_last_li, msg_data_li})
     ,.v_o(msg_data_v_li)
     ,.yumi_i(msg_data_yumi_lo)
     );

  wire [stream_cnt_width_lp-1:0] stream_size =
    `BSG_MAX((1'b1 << msg_header_li.size) / stream_bytes_lp, 1'b1) - 1'b1;
  wire nz_stream = stream_size > '0;
  wire fsm_stream = fsm_stream_mask_p[msg_header_li.msg_type];
  wire msg_stream = msg_stream_mask_p[msg_header_li.msg_type];

  logic cnt_up;
  wire [stream_cnt_width_lp-1:0] size_li = fsm_stream ? stream_size : '0;
  wire [stream_cnt_width_lp-1:0] first_cnt = msg_header_li.addr[stream_offset_width_lp+:stream_cnt_width_lp];
  bp_me_burst_pump_control
   #(.max_val_p(stream_words_lp-1))
   pump_control
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.en_i(cnt_up)
     ,.size_i(size_li)
     ,.val_i(first_cnt)

     ,.wrap_o(fsm_cnt_o)
     ,.first_o(fsm_new_o)
     ,.last_o(fsm_last_o)
     );

  wire [paddr_width_p-1:0] wrap_addr =
    {msg_header_li.addr[paddr_width_p-1:block_offset_width_lp]
     ,{stream_words_lp>1{fsm_cnt_o}}
     ,msg_header_li.addr[0+:stream_offset_width_lp]
     };

  always_comb
    begin
      fsm_header_cast_o = msg_header_li;
      // keep the address to be the critical word address
      fsm_header_cast_o.addr[0+:block_offset_width_lp] = msg_header_li.addr;
      fsm_data_o = msg_data_li;

      if (~msg_stream & fsm_stream)
        begin
          // 1:N
          // convert one msg message into stream of N FSM messages
          fsm_v_o = msg_header_v_li & (msg_data_v_li | ~msg_has_data_li);
          msg_header_yumi_lo = fsm_yumi_i & fsm_last_o & msg_header_v_li;
          msg_data_yumi_lo = fsm_yumi_i & fsm_last_o & msg_data_v_li;
          cnt_up = fsm_yumi_i;
          fsm_addr_o = wrap_addr;
        end
      else if (msg_stream & ~fsm_stream)
        begin
          // N:1
          // consume all but last msg input beat silently, then FSM consumes last beat
          fsm_v_o = msg_header_v_li & ((msg_data_v_li & msg_last_li) | ~msg_has_data_li);
          msg_header_yumi_lo = fsm_yumi_i & fsm_last_o;
          msg_data_yumi_lo = msg_data_v_i & (~fsm_last_o | fsm_yumi_i);
          cnt_up = msg_data_yumi_lo;
          // Hold address constant at critical address
          fsm_addr_o = msg_header_li.addr;
        end
      else
        begin
          // 1:1
          fsm_v_o = msg_header_v_li & (msg_data_v_li | ~msg_has_data_li);
          msg_header_yumi_lo = msg_header_v_li & fsm_yumi_i & fsm_last_o;
          msg_data_yumi_lo = msg_data_v_li & fsm_yumi_i;
          cnt_up = fsm_yumi_i;
          fsm_addr_o = wrap_addr;
        end
    end

  // parameter checks
  if (block_width_p % stream_data_width_p != 0)
    $error("block_width_p must be evenly divisible by stream_data_width_p");
  if (block_width_p < stream_data_width_p)
    $error("block_width_p must be at least as large as stream_data_width_p");

endmodule

`BSG_ABSTRACT_MODULE(bp_me_burst_pump_in)

