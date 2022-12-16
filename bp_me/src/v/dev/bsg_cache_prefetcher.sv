/**
 * bsg_cache_prefetcher.sv
 *
 * @author Aaron Chang
 */

`include "bsg_defines.v"
`include "bsg_cache.vh"

module bsg_cache_prefetcher
  import bsg_cache_pkg::*;
  #(parameter `BSG_INV_PARAM(addr_width_p)
    ,parameter `BSG_INV_PARAM(data_width_p)
    ,parameter `BSG_INV_PARAM(block_size_in_words_p)
    ,parameter `BSG_INV_PARAM(buffers_p) // currently only supports 1, future implementation should be in powers of 2
    ,parameter `BSG_INV_PARAM(buffer_depth_p)
    ,parameter `BSG_INV_PARAM(stride_p)
    ,parameter burst_len_lp=(block_size_in_words_p*data_width_p/data_width_p)
    ,parameter dma_pkt_width_lp=`bsg_cache_dma_pkt_width(addr_width_p)
    // For future use for multiple buffers implementation
    ,parameter lg_buffers_lp=`BSG_SAFE_CLOG2(buffers_p)
    ,parameter lg_buffer_depth_lp=`BSG_SAFE_CLOG2(buffer_depth_p)
  )
  (
    input clk_i
    ,input reset_i

    // upstream from cache
    ,input [dma_pkt_width_lp-1:0] up_pkt_i
    ,input up_pkt_v_i
    ,output up_pkt_ready_o

    ,output logic [data_width_p-1:0] up_data_o
    ,output logic up_data_v_o
    ,input up_data_yumi_i

    ,input [data_width_p-1:0] up_data_i
    ,input up_data_v_i
    ,output logic up_data_ready_o

    // downstream to DMA
    ,output logic [dma_pkt_width_lp-1:0] down_pkt_o
    ,output logic down_pkt_v_o
    ,input down_pkt_yumi_i

    ,input [data_width_p-1:0] down_data_i
    ,input down_data_v_i
    ,output logic down_data_ready_o

    ,output logic [data_width_p-1:0] down_data_o
    ,output logic down_data_v_o
    ,input down_data_yumi_i
  );

  localparam counter_width_lp=`BSG_SAFE_CLOG2(burst_len_lp+1);

  // dma states
  typedef enum logic [2:0] {
    IDLE_DMA
    ,READ_DMA
    ,WRITE_DMA
    ,PREFETCH
  } dma_state_e;
  dma_state_e dma_state_n;
  dma_state_e dma_state_r;

  // dma packet
  `declare_bsg_cache_dma_pkt_s(addr_width_p);
  bsg_cache_dma_pkt_s up_pkt;
  assign up_pkt = up_pkt_i;
  bsg_cache_dma_pkt_s prefetch_pkt_lo;
  assign prefetch_pkt_lo.write_not_read = 1'b0; // prefetch will always only be reads
  logic prefetch_pkt_v_lo;
  logic prefetch_pkt_yumi_li;

  // dma counter
  logic counter_clear;
  logic counter_up;
  logic [counter_width_lp-1:0] counter_r;

  bsg_counter_clear_up #(
    .max_val_p(burst_len_lp)
   ,.init_val_p('0)
  ) dma_counter (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.clear_i(counter_clear)
    ,.up_i(counter_up)
    ,.count_o(counter_r)
  );

  wire counter_fill_max = counter_r == (burst_len_lp-1);
  wire counter_evict_max = counter_r == burst_len_lp;

  //TODO: support multiple buffers and update logic to pick which FIFO to flush
  logic flush_fifo;
  bsg_edge_detect #(
    .falling_not_rising_p(0)
  ) read_rising_edge (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.sig_i(dma_state_r == READ_DMA)
    ,.detect_o(flush_fifo)
  );

  bsg_edge_detect #(
    .falling_not_rising_p(0)
  ) pref_rising_edge (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.sig_i(prefetching_sel)
    ,.detect_o(prefetch_pkt_v_lo)
  );

  logic prefetching_done;
  bsg_edge_detect #(
    .falling_not_rising_p(1)
  ) pref_falling_edge (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.sig_i(prefetching_sel)
    ,.detect_o(prefetching_done)
  );

  // Prefetch FIFO
  logic prefetch_fifo_v_li;
  logic prefetch_fifo_ready_lo;
  logic [addr_width_p-1:0] prefetch_fifo_data_li;
  logic prefetch_fifo_v_lo;

  //TODO: support multiple buffers and use round robin arb to select which to pull from
  bsg_fifo_1r1w_small #(
    .width_p(addr_width_p)
    ,.els_p(buffer_depth_p)
  ) prefetch_fifo (
    .clk_i(clk_i)
    ,.reset_i(reset_i | flush_fifo)
    ,.v_i(prefetch_fifo_v_li)
    ,.ready_o(prefetch_fifo_ready_lo)
    ,.data_i(prefetch_fifo_data_li)
    ,.v_o(prefetch_fifo_v_lo)
    ,.data_o(prefetch_pkt_lo.addr)
    ,.yumi_i(prefetching_done)
  );
  
  logic [addr_width_p-1:0] next_addr_reg_li;
  assign next_addr_reg_li = flush_fifo ? up_pkt.addr+stride_p : prefetch_fifo_data_li+stride_p;
  assign prefetch_fifo_v_li = (dma_state_r == READ_DMA | prefetch_fifo_v_lo | prefetching_done) & prefetch_fifo_ready_lo;

  bsg_dff_en #(
    .width_p(addr_width_p)
  ) next_addr_reg
    (.clk_i  (clk_i)
    ,.data_i (next_addr_reg_li)
    ,.en_i   (prefetch_fifo_v_li | flush_fifo)
    ,.data_o (prefetch_fifo_data_li)
  );

  logic streaming_sel;
  logic stream_ready_lo;
  bsg_edge_detect #(
    .falling_not_rising_p(0)
  ) stream_rising_edge (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.sig_i(streaming_sel)
    ,.detect_o(stream_ready_lo)
  );

  logic stream_done;
  bsg_edge_detect #(
    .falling_not_rising_p(1)
  ) stream_falling_edge (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.sig_i(streaming_sel)
    ,.detect_o(stream_done)
  );

  // DMA packet interface
  logic address_hit;
  logic [addr_width_p-1:0] addr_fifo_data_lo;
  
  assign address_hit = up_pkt.addr == addr_fifo_data_lo;
  assign prefetching_sel = dma_state_r == PREFETCH;
  assign down_pkt_o = prefetching_sel ? prefetch_pkt_lo : up_pkt_i;
  assign down_pkt_v_o = prefetching_sel ? prefetch_pkt_v_lo: up_pkt_v_i & !streaming_sel;
  assign up_pkt_ready_o = prefetching_sel ? address_hit & up_pkt_v_i : (streaming_sel ? stream_ready_lo : down_pkt_yumi_i);
  assign prefetch_pkt_yumi_li = prefetching_sel & down_pkt_yumi_i;

  // Buffer address storage
  bsg_fifo_1r1w_small #(
    .width_p(addr_width_p)
    ,.els_p(buffer_depth_p+1) // Add 1 to support incoming/outgoing buffer data at the same time
  ) buffer_addr_fifo (
    .clk_i(clk_i)
    ,.reset_i(reset_i | flush_fifo)
    ,.v_i(prefetch_pkt_v_lo)
    ,.ready_o()
    ,.data_i(prefetch_pkt_lo.addr)
    ,.v_o()
    ,.data_o(addr_fifo_data_lo)
    ,.yumi_i(stream_done)
  );

  // Buffer data storage
  logic data_fifo_v_li;
  logic data_fifo_ready_lo;
  logic data_fifo_v_lo;
  logic [data_width_p-1:0] data_fifo_data_lo;

  //TODO: support multiple buffers and associate data with addresses (using CAM with FIFOs?)
  bsg_fifo_1r1w_small #(
    .width_p(data_width_p)
    ,.els_p((burst_len_lp<2) ? 2*buffer_depth_p : burst_len_lp*buffer_depth_p)
  ) buffer_data_fifo (
    .clk_i(clk_i)
    ,.reset_i(reset_i | flush_fifo)
    ,.v_i(data_fifo_v_li)
    ,.ready_o(data_fifo_ready_lo)
    ,.data_i(down_data_i)
    ,.v_o(data_fifo_v_lo)
    ,.data_o(data_fifo_data_lo)
    ,.yumi_i(data_fifo_yumi_li)
  );

  assign data_fifo_v_li = down_data_v_i & prefetching_sel;

  // Cache read data interface
  assign up_data_o = streaming_sel ? data_fifo_data_lo : down_data_i & data_width_p'(!prefetching_sel);
  assign up_data_v_o = streaming_sel ? data_fifo_v_lo : down_data_v_i & !prefetching_sel;
  assign down_data_ready_o = prefetching_sel ? data_fifo_ready_lo : up_data_yumi_i & !streaming_sel;
  assign data_fifo_yumi_li = streaming_sel & up_data_yumi_i;

  // Cache write data interface
  assign down_data_v_o = up_data_v_i & !prefetching_sel;
  assign down_data_o = up_data_i;
  assign up_data_ready_o = down_data_yumi_i & !prefetching_sel;

  logic read_done;
  bsg_edge_detect #(
    .falling_not_rising_p(1)
  ) read_falling_edge (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.sig_i(up_data_v_o)
    ,.detect_o(read_done)
  );

  logic write_done;
  bsg_edge_detect #(
    .falling_not_rising_p(1)
  ) write_falling_edge (
    .clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.sig_i(up_data_v_i)
    ,.detect_o(write_done)
  );

  logic read_req;
  logic write_req;
  assign streaming_sel = read_req & data_fifo_v_lo & address_hit;

  always_comb begin
    if (up_pkt_v_i && !up_pkt.write_not_read) begin
      read_req = 1'b1;
    end else if (up_pkt_v_i && up_pkt.write_not_read) begin
      write_req = 1'b1;
    end else if (read_done) begin
      read_req = 1'b0;
    end else if (write_done) begin
      write_req = 1'b0;
    end

    case (dma_state_r)
      IDLE_DMA: begin
        if (read_req && !(data_fifo_v_lo && address_hit)) begin
          dma_state_n = READ_DMA;
          counter_clear = 1'b1;
        end else if (write_req) begin
          dma_state_n = WRITE_DMA;
        end else if (prefetch_fifo_v_lo & data_fifo_ready_lo & prefetch_pkt_lo.addr !== addr_fifo_data_lo) begin
          dma_state_n = PREFETCH;
          counter_clear = 1'b1;
        end else begin
          dma_state_n = IDLE_DMA;
        end
      end
      READ_DMA: begin
        counter_up = down_data_v_i & ~counter_fill_max;
        counter_clear = down_data_v_i & counter_fill_max;
        if (counter_fill_max & down_data_v_i) begin // read data received
          dma_state_n = IDLE_DMA;
        end else begin
          dma_state_n = READ_DMA;
        end
      end
      WRITE_DMA: begin
        if (write_done) begin // write data transferred
          dma_state_n = IDLE_DMA;
        end else begin
          dma_state_n = WRITE_DMA;
        end
      end
      PREFETCH: begin
        counter_up = down_data_v_i & ~counter_fill_max;
        counter_clear = down_data_v_i & counter_fill_max;
        if (counter_fill_max & down_data_v_i) begin // prefetched data received
          dma_state_n = IDLE_DMA;
        end else begin
          dma_state_n = PREFETCH;
        end
      end
      default: begin
        // this should never happen, but if it does, then go back to IDLE_DMA.
        dma_state_n = IDLE_DMA;
        counter_up = 1'b0;
      end
    endcase
  end

  always_ff @ (posedge clk_i) begin
    if (reset_i) begin
      dma_state_r <= IDLE_DMA;
    end
    else begin
      dma_state_r <= dma_state_n;
    end
  end

endmodule
