// 25 MHz pixel-domain PCM pacing and held ready/valid output.
// One 48 kHz demand reads one FIFO sample, or queues silence on underflow.
module pcm_output_ctrl (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire               fifo_empty,
    input  wire               pcm_out_valid,
    input  wire signed [15:0] pcm_out_data,
    output wire               pcm_rd_en,
    output wire               sample_valid,
    input  wire               sample_ready,
    output wire signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output reg                underflow,
    output reg                output_stall,
    output wire               sample_consume_done,
    output wire               pcm_drained
);

    // The remainder stays below 25,000,000; the widened sum cannot wrap.
    reg [24:0] sample_phase;
    wire [25:0] phase_sum = {1'b0, sample_phase} + 26'd48000;
    wire sample_tick = (phase_sum >= 26'd25000000);
    wire [25:0] phase_remainder = phase_sum - 26'd25000000;

    reg signed [15:0] sample_queue [0:7];
    reg [2:0] write_ptr;
    reg [2:0] read_ptr;
    reg [3:0] queue_count;
    reg rd_pending;
    // Tags distinguish actual PCM zero samples from generated silence.
    reg content_queue [0:7];
    reg [3:0] content_count;

    assign sample_valid = (queue_count != 4'd0);
    assign sample_left = sample_valid ? sample_queue[read_ptr] : 16'sd0;
    assign sample_right = sample_left;

    wire queue_pop = sample_valid && sample_ready;
    assign sample_consume_done = queue_pop && content_queue[read_ptr];
    assign pcm_drained = fifo_empty && (content_count == 4'd0) &&
                         !rd_pending && !pcm_out_valid && !pcm_rd_en;
    // A previously launched read owns one slot until its registered response
    // is sampled here. A same-edge output acceptance releases one slot.
    wire [4:0] reserved_count = {1'b0, queue_count} +
                               (rd_pending ? 5'd1 : 5'd0);
    wire [4:0] occupied_after_pop = reserved_count -
                                   (queue_pop ? 5'd1 : 5'd0);
    wire reserve_available = (occupied_after_pop < 5'd8);
    wire sample_demand = enable && sample_tick;
    wire new_slot_available = reserve_available && !rd_pending;

    assign pcm_rd_en = rst_n && sample_demand && new_slot_available &&
                       !fifo_empty;
    wire silence_push = sample_demand && new_slot_available && fifo_empty;
    // The response is accepted even if enable has fallen since the request.
    // Ignore an unsolicited response; never overwrite an occupied queue slot.
    wire response_push = rd_pending && pcm_out_valid &&
                         ((queue_count < 4'd8) || queue_pop);
    wire queue_push = response_push || silence_push;
    wire signed [15:0] push_data = response_push ? pcm_out_data : 16'sd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_phase <= 25'd0;
            write_ptr <= 3'd0;
            read_ptr <= 3'd0;
            queue_count <= 4'd0;
            rd_pending <= 1'b0;
            content_count <= 4'd0;
            underflow <= 1'b0;
            output_stall <= 1'b0;
        end else begin
            if (sample_tick)
                sample_phase <= phase_remainder[24:0];
            else
                sample_phase <= phase_sum[24:0];

            rd_pending <= pcm_rd_en;

            if (queue_push) begin
                sample_queue[write_ptr] <= push_data;
                content_queue[write_ptr] <= response_push;
                write_ptr <= write_ptr + 3'd1;
            end
            if (queue_pop)
                read_ptr <= read_ptr + 3'd1;

            case ({queue_push, queue_pop})
                2'b10: queue_count <= queue_count + 4'd1;
                2'b01: queue_count <= queue_count - 4'd1;
                default: queue_count <= queue_count;
            endcase

            case ({response_push, sample_consume_done})
                2'b10: content_count <= content_count + 4'd1;
                2'b01: content_count <= content_count - 4'd1;
                default: content_count <= content_count;
            endcase

            if (silence_push)
                underflow <= 1'b1;
            if (sample_demand && !new_slot_available)
                output_stall <= 1'b1;
        end
    end

endmodule
