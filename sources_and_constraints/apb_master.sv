`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: apb_master
// Description: Organisational wrapper (apb_spec_v1 sec 2 / sequencer_top sec 9):
//   apb_master_mac (application) + apb_master_phy (APB3 protocol). Adds NO
//   logic. Presents: the seq_top_fsm request face, the tx_msg_arbiter read-
//   reply handshake, phy_prdata to the composer (datapath, combinational from
//   the PHY's prdata_o register), and one APB bus toward apb_interconnect.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module apb_master #(
    parameter int unsigned ADDR_W = RGF_ADDR_W,
    parameter int unsigned SEL_W  = RGF_SEL_W
)(
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- from seq_top_fsm / classifier ----
    input  logic        req,
    input  logic        is_write,
    input  logic [LOCAL_ADDR_WIDTH-1:0] local_addr,
    input  logic [BAR_ADDR_WIDTH-1:0]   bar_addr,
    input  logic [SEL_W-1:0]            rgf_sel,
    input  logic [WDATA_WIDTH-1:0]      wdata,
    output logic        done,

    // ---- read-reply handshake (tx_msg_arbiter) + composer datapath ----
    output logic        rd_resp_valid,
    input  logic        resp_captured,
    output logic [31:0] phy_prdata,       // = PHY prdata_o -> composer rdata

    // ---- APB bus (to apb_interconnect) ----
    output logic              psel,
    output logic              penable,
    output logic [ADDR_W-1:0] paddr,
    output logic              pwrite,
    output logic [31:0]       pwdata,
    output logic [SEL_W-1:0]  slave_sel,
    input  logic [31:0]       prdata,
    input  logic              pready,
    output logic              rgf_r0c_read  // -> R0C registers (uart_rgf)
);

    logic        transfer, phy_done;
    logic [ADDR_W-1:0] mac_paddr;
    logic        mac_pwrite;
    logic [31:0] mac_pwdata;

    apb_master_mac #(.ADDR_W(ADDR_W), .SEL_W(SEL_W)) u_mac (
        .clk(clk), .reset_n(reset_n), .enable(enable),
        .req(req), .is_write(is_write),
        .local_addr(local_addr), .bar_addr(bar_addr),
        .rgf_sel(rgf_sel), .wdata(wdata),
        .phy_done(phy_done), .phy_prdata(phy_prdata),
        .transfer(transfer), .paddr(mac_paddr), .pwrite(mac_pwrite),
        .pwdata(mac_pwdata),
        .slave_sel(slave_sel),
        .rd_resp_valid(rd_resp_valid), .resp_captured(resp_captured),
        .done(done)
    );

    apb_master_phy #(.ADDR_W(ADDR_W)) u_phy (
        .clk(clk), .reset_n(reset_n), .enable(enable),
        .transfer(transfer),
        .paddr_i(mac_paddr), .pwrite_i(mac_pwrite), .pwdata_i(mac_pwdata),
        .psel_o(psel), .penable_o(penable), .paddr_o(paddr),
        .pwrite_o(pwrite), .pwdata_o(pwdata),
        .pready_i(pready), .prdata_i(prdata),
        .prdata_o(phy_prdata), .phy_done(phy_done),
        .rgf_r0c_read(rgf_r0c_read)
    );

endmodule
