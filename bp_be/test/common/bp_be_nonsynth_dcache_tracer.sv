
`include "bp_common_defines.svh"
`include "bp_top_defines.svh"

module bp_be_nonsynth_dcache_tracer
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 #( parameter bp_params_e bp_params_p = e_bp_default_cfg
  , parameter assoc_p = 8
  , parameter sets_p = 64
  , parameter block_width_p = 512
  , parameter fill_width_p = 512
  , parameter trace_file_p = "dcache"
   `declare_bp_proc_params(bp_params_p)
   `declare_bp_cache_engine_if_widths(paddr_width_p, ctag_width_p, sets_p, assoc_p, dword_width_gp, block_width_p, fill_width_p, cache)

   // Calculated parameters
   , localparam mhartid_width_lp = `BSG_SAFE_CLOG2(num_core_p)
   , localparam bank_width_lp = block_width_p / assoc_p
   , localparam dcache_pkt_width_lp = $bits(bp_be_dcache_pkt_s)
   , localparam wbuf_entry_width_lp = `bp_be_dcache_wbuf_entry_width(paddr_width_p, assoc_p)
   )
  (  input                                                clk_i
   , input                                                reset_i
   , input                                                freeze_i
   , input [mhartid_width_lp-1:0]                         mhartid_i

   , input [dcache_pkt_width_lp-1:0]                      dcache_pkt_i
   , input                                                v_i
   , input                                                ready_o

   , input [dpath_width_gp-1:0]                           early_data_o
   , input                                                early_v_o
   , input [dpath_width_gp-1:0]                           final_data_o
   , input                                                final_v_o

   , input [cache_req_width_lp-1:0]                       cache_req_o
   , input                                                cache_req_v_o
   , input                                                cache_req_yumi_i
   , input                                                cache_req_busy_i
   , input [cache_req_metadata_width_lp-1:0]              cache_req_metadata_o
   , input                                                cache_req_metadata_v_o
   , input                                                cache_req_critical_tag_i
   , input                                                cache_req_critical_data_i
   , input                                                cache_req_complete_i
   // Unused
   , input                                                cache_req_credits_full_i
   , input                                                cache_req_credits_empty_i

   , input                                                data_mem_pkt_v_i
   , input [cache_data_mem_pkt_width_lp-1:0]              data_mem_pkt_i
   , input                                                data_mem_pkt_yumi_o
   , input [block_width_p-1:0]                            data_mem_o

   , input                                                tag_mem_pkt_v_i
   , input [cache_tag_mem_pkt_width_lp-1:0]               tag_mem_pkt_i
   , input                                                tag_mem_pkt_yumi_o
   , input [cache_tag_info_width_lp-1:0]                  tag_mem_o

   , input                                                stat_mem_pkt_v_i
   , input [cache_stat_mem_pkt_width_lp-1:0]              stat_mem_pkt_i
   , input                                                stat_mem_pkt_yumi_o
   , input [cache_stat_info_width_lp-1:0]                 stat_mem_o

   , input [wbuf_entry_width_lp-1:0]                      wbuf_entry_out
   , input                                                wbuf_yumi_li

   , input bp_be_dcache_decode_s                          decode_tl_r
   , input [paddr_width_p-1:0]                            paddr_tl
   , input bp_be_dcache_decode_s                          decode_tv_r
   , input [paddr_width_p-1:0]                            paddr_tv_r
   , input [dword_width_gp-1:0]                           st_data_tv_r
   , input bp_be_dcache_decode_s                          decode_dm_r
   );

  `declare_bp_cache_engine_if(paddr_width_p, ctag_width_p, sets_p, assoc_p, dword_width_gp, block_width_p, fill_width_p, cache);
  bp_be_dcache_pkt_s dcache_pkt_cast_i;
  assign dcache_pkt_cast_i = dcache_pkt_i;

  bp_cache_req_s cache_req_cast_o;
  bp_cache_req_metadata_s cache_req_metadata_cast_o;
  assign cache_req_cast_o = cache_req_o;
  assign cache_req_metadata_cast_o = cache_req_metadata_o;

  bp_cache_data_mem_pkt_s data_mem_pkt_cast_i;
  bp_cache_tag_mem_pkt_s tag_mem_pkt_cast_i;
  bp_cache_stat_mem_pkt_s stat_mem_pkt_cast_i;
  assign data_mem_pkt_cast_i = data_mem_pkt_i;
  assign tag_mem_pkt_cast_i = tag_mem_pkt_i;
  assign stat_mem_pkt_cast_i = stat_mem_pkt_i;

  logic [assoc_p-1:0][bank_width_lp-1:0] data_mem_cast_o;
  bp_cache_tag_info_s tag_mem_info_cast_o;
  bp_cache_tag_info_s stat_mem_info_cast_o;
  assign data_mem_cast_o = data_mem_o;
  assign tag_mem_info_cast_o = tag_mem_o;
  assign stat_mem_info_cast_o = stat_mem_o;

  `declare_bp_be_dcache_wbuf_entry_s(paddr_width_p, assoc_p);
  bp_be_dcache_wbuf_entry_s wbuf_entry_out_cast;
  assign wbuf_entry_out_cast = wbuf_entry_out;

  integer file;
  string file_name;
  wire delay_li = reset_i | freeze_i;
  always_ff @(negedge delay_li)
   begin
     file_name = $sformatf("%s_%x.trace", trace_file_p, mhartid_i);
     file      = $fopen(file_name, "w");
     $fwrite(file, "Coherent L1: %x\n", l1_coherent_p);
   end

  logic data_mem_read_r, tag_mem_read_r, stat_mem_read_r;
  always_ff @(posedge clk_i)
    if (reset_i)
      begin
        data_mem_read_r <= '0;
        tag_mem_read_r <= '0;
        stat_mem_read_r <= '0;
      end
    else
      begin
        data_mem_read_r <= data_mem_pkt_yumi_o & (data_mem_pkt_cast_i.opcode == e_cache_data_mem_read);
        tag_mem_read_r <= tag_mem_pkt_yumi_o & (tag_mem_pkt_cast_i.opcode == e_cache_tag_mem_read);
        stat_mem_read_r <= stat_mem_pkt_yumi_o & (stat_mem_pkt_cast_i.opcode == e_cache_stat_mem_read);
      end

  always_ff @(posedge clk_i)
    begin
      if (ready_o & v_i)
        $fwrite(file, "[%t] access: %p\n", $time, dcache_pkt_cast_i);

      if (data_mem_pkt_yumi_o)
        $fwrite(file, "[%t] data_mem_pkt: %p\n", $time, data_mem_pkt_cast_i);
      if (data_mem_read_r)
        $fwrite(file, "[%t] data_mem_read: %x\n", $time, data_mem_cast_o);

      if (tag_mem_pkt_yumi_o)
        $fwrite(file, "[%t] tag_mem_pkt: %p\n", $time, tag_mem_pkt_cast_i);
      if (tag_mem_read_r)
        $fwrite(file, "[%t] tag_mem_read: %x\n", $time, tag_mem_info_cast_o);

      if (stat_mem_pkt_yumi_o)
        $fwrite(file, "[%t] stat_mem_pkt: %p\n", $time, stat_mem_pkt_cast_i);
      if (stat_mem_read_r)
        $fwrite(file, "[%t] stat_mem_read: %x\n", $time, stat_mem_info_cast_o);

      if (early_v_o & decode_tv_r.load_op)
        $fwrite(file, "[%t] early load: [%x]->%x\n", $time, paddr_tv_r, early_data_o);
      if (early_v_o & decode_tv_r.store_op)
        $fwrite(file, "[%t] early store: [%x]<-%x\n", $time, paddr_tv_r, st_data_tv_r);

      if (final_v_o & decode_dm_r.load_op)
        $fwrite(file, "[%t] final load: %x\n", $time, final_data_o);

      if (cache_req_yumi_i)
        $fwrite(file, "[%t] cache_req: %p\n", $time, cache_req_cast_o);

      if (cache_req_metadata_v_o)
        $fwrite(file, "[%t] cache_req_metadata: %p\n", $time, cache_req_metadata_cast_o);

      if (cache_req_critical_tag_i)
        $fwrite(file, "[%t] cache_req_critical_tag_i raised\n", $time);

      if (cache_req_critical_data_i)
        $fwrite(file, "[%t] cache_req_critical_data_i raised\n", $time);

      if (cache_req_complete_i)
        $fwrite(file, "[%t] cache_req_complete_i raised\n", $time);

      if (wbuf_yumi_li)
        $fwrite(file, "[%t] wbuf: %p\n", $time, wbuf_entry_out_cast);
    end

endmodule

