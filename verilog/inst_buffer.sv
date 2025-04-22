module inst_buffer #(
    parameter DEPTH = 16,
    parameter WIDTH = 64,
    parameter ADDR  = $clog2(DEPTH)
) (
    input                   clock,
    input                   reset,

    input                   squash,
    input  [`XLEN-1:0]      branch_target,

    input                   read_enable,
    input  IF_IB_PACKET     if_ib_packet,
    input ack,//todo? do we need ack for controlling enable?

    output IB_ID_PACKET     ib_id_packet,
    output logic            enable,
    output                  IsFull
);


    IF_IB_PACKET        buffer[0:DEPTH-1];
    logic [ADDR:0]      w_ptr;
    logic [ADDR:0]      r_ptr;
    logic               w_en, r_en;//todo
    IB_ID_PACKET        r_data;
    logic               full, empty;

    assign IsFull = full;
    assign full  = (w_ptr[ADDR] == ~r_ptr[ADDR]) && (w_ptr[ADDR-1:0] == r_ptr[ADDR-1:0]);
    assign empty = (w_ptr == r_ptr)? 1'b1 : 1'b0;
    assign w_en  = if_ib_packet.valid;
    assign r_en  = read_enable;//todo

    assign ib_id_packet = r_data;

    always_ff @( posedge clock ) begin : update
        if(reset) begin
            for (integer i = 0; i < DEPTH; i = i + 1) begin
                buffer[i] <= 0;
            end
            w_ptr <= 'd0;
        end else if(squash) begin
            w_ptr <= 0;
        end else if(w_en & !full) begin
            buffer[w_ptr[ADDR-1:0]] <= if_ib_packet;
            w_ptr <= w_ptr + 1;
        end
    end
    
    always_ff @( posedge clock ) begin : read
        if (reset) begin
            r_data <= 0;
            r_ptr <= 0;
            enable <= 0;
        end else if(squash) begin
            r_ptr <= 0;
            enable <= 0;
            r_data <= 0;
        end
        else if((r_en) && (!empty)) begin
            r_data.valid <= buffer[r_ptr[ADDR-1:0]].valid;
            r_data.inst  <= buffer[r_ptr[ADDR-1:0]].inst;
            r_data.NPC   <= buffer[r_ptr[ADDR-1:0]].NPC;
            r_data.PC    <= buffer[r_ptr[ADDR-1:0]].PC;
            enable       <= 1 && (buffer[r_ptr[ADDR-1:0]].inst != 32'b0);
            r_ptr        <= r_ptr + 1;
        end else if(ack) begin
            r_data.valid <= 0;
            enable <= 0;
        end
    end


    
endmodule


// module inst_buffer #(
//     parameter DEPTH = 16,
//     parameter WIDTH = 64,
//     parameter ADDR  = $clog2(DEPTH)
// ) (
//     input                   clock,
//     input                   reset,

//     input                   squash,
//     input  [`XLEN-1:0]      branch_target,

//     input read_enable,
//     input  IF_IB_PACKET     if_ib_packet[0:1],
//     // input [`XLEN-1:0]       PredictionPC,
//     // input                   predict_taken,
//     input ack,//todo? do we need ack for controlling enable?

//     output IB_ID_PACKET     ib_id_packet,
//     output logic enable
// );


//     IF_IB_PACKET        buffer[0:DEPTH-1][0:1];
//     logic [ADDR:0]      w_ptr;
//     logic [ADDR:0]      r_ptr;
//     logic               w_en, r_en;//todo
//     IB_ID_PACKET        r_data;
//     logic               full, empty;

//     assign full  = (w_ptr[ADDR] == ~r_ptr[ADDR]) && (w_ptr[ADDR-1:0] == r_ptr[ADDR-1:0]);
//     assign empty = (w_ptr == r_ptr)? 1'b1 : 1'b0;
//     assign w_en  = if_ib_packet[0].valid;
//     assign r_en  = read_enable;//todo

//     assign ib_id_packet = r_data;

//     always_ff @( posedge clock ) begin : update
//         if(reset) begin
//             for (integer i = 0; i < DEPTH; i = i + 1) begin
//                 buffer[i][0] <= 0;
//                 buffer[i][1] <= 0;
//             end
//             w_ptr <= 'd0;
//         end else if(squash) begin
//             w_ptr <= 0;
//         end else if(w_en & !full) begin
//             buffer[w_ptr[ADDR-1:0]] <= if_ib_packet;
//             w_ptr <= w_ptr + 1;
//         end
//     end
    
//     logic read_phase;
//     logic ready;
//     always_ff @( posedge clock ) begin : read
//         if (reset) begin
//             r_data <= 0;
//             r_ptr <= 0;
//             read_phase <= 0;
//             enable <= 0;
//             ready <= 1;
//         end else if(squash) begin
//             if (branch_target[2] == 1) begin
//                 read_phase <= 1;
//             end else begin
//                 read_phase <= 0;
//             end
//             r_ptr <= 0;
//             enable <= 0;
//             ready <= 1;
//             r_data <= 0;
//         end
//         // else if (predict_taken) begin
//         //     if (PredictionPC[2] == 1) begin
//         //         read_phase <= 1;
//         //     end else begin
//         //         read_phase <= 0;
//         //     end
//         //     enable <= 0;
//         //     ready <= 1;
//         //     r_data <= 0;
//         // end
//         else if((r_en) && (!empty) && (read_phase == 0)) begin
//             r_data.valid <= buffer[r_ptr[ADDR-1:0]][0].valid;
//             r_data.inst  <= buffer[r_ptr[ADDR-1:0]][0].inst;
//             r_data.NPC   <= buffer[r_ptr[ADDR-1:0]][0].NPC;
//             r_data.PC    <= buffer[r_ptr[ADDR-1:0]][0].PC;
//             read_phase <= 1;
//             enable <= 1 && (buffer[r_ptr[ADDR-1:0]][0].inst != 32'b0);
//             ready <= 0;
//         end else if((r_en) && (!empty) && (read_phase == 1)) begin
//             r_data.valid <= buffer[r_ptr[ADDR-1:0]][1].valid;
//             r_data.inst  <= buffer[r_ptr[ADDR-1:0]][1].inst;
//             r_data.NPC   <= buffer[r_ptr[ADDR-1:0]][1].NPC;
//             r_data.PC    <= buffer[r_ptr[ADDR-1:0]][1].PC;
//             read_phase <= 0;
//             enable <= 1 && (buffer[r_ptr[ADDR-1:0]][1].inst != 32'b0);
//             r_ptr <= r_ptr + 1;
//             ready <= 0;
//         end else if(ack) begin
//             r_data.valid <= 0;
//             enable <= 0;
//         end else begin
//             ready <= 1;
//         end
//     end


    
// endmodule