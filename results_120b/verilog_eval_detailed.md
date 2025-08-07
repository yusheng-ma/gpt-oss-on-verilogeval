# Verilog Evaluation Detailed Summary

### Prob001_zero
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - output zero

The module should always outputs a LOW.
```
</details>

#### reference
```verilog
module RefModule (
  output zero
);

  assign zero = 1'b0;

endmodule
```

#### model output
```verilog
module TopModule (
    output zero
);
    assign zero = 1'b0;
endmodule
```
---

### Prob002_m2014_q4i
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - output out

The module should always drive 0 (or logic low).
```
</details>

#### reference
```verilog
module RefModule (
  output out
);

  assign out = 1'b0;

endmodule
```

#### model output
```verilog
module TopModule (
    output out
);
    assign out = 1'b0;
endmodule
```
---

### Prob003_step_one
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - output one

The module should always drive 1 (or logic high).
```
</details>

#### reference
```verilog
module RefModule (
  output one
);

  assign one = 1'b1;

endmodule
```

#### model output
```verilog
module TopModule (
    output one
);
    assign one = 1'b1;
endmodule
```
---

### Prob004_vector2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (32 bits)
 - output out (32 bits)

The module should reverse the byte order of a 32-bit vector.
```
</details>

#### reference
```verilog
module RefModule (
  input [31:0] in,
  output [31:0] out
);

  assign out = {in[7:0], in[15:8], in[23:16], in[31:24]};

endmodule
```

#### model output
```verilog
module TopModule (
    input  [31:0] in,
    output [31:0] out
);
    assign out = {in[7:0], in[15:8], in[23:16], in[31:24]};
endmodule
```
---

### Prob005_notgate
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in
 - output out

The module should implement a NOT gate.
```
</details>

#### reference
```verilog
module RefModule (
  input in,
  output out
);

  assign out = ~in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  in,
    output out
);
    assign out = ~in;
endmodule
```
---

### Prob006_vectorr
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (8 bits)
 - output out (8 bits)

The module should reverse the bit ordering of the input port and write
the result to the output port.
```
</details>

#### reference
```verilog
module RefModule (
  input [7:0] in,
  output [7:0] out
);

  assign {out[0],out[1],out[2],out[3],out[4],out[5],out[6],out[7]} = in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [7:0] in,
    output wire [7:0] out
);
    assign out = {in[0], in[1], in[2], in[3], in[4], in[5], in[6], in[7]};
endmodule
```
---

### Prob007_wire
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in
 - output out

The module should behave like a wire.
```
</details>

#### reference
```verilog
module RefModule (
  input in,
  output out
);

  assign out = in;

endmodule
```

#### model output
```verilog
module TopModule(
    input  in,
    output out
);
    assign out = in;
endmodule
```
---

### Prob008_m2014_q4h
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in
 - output out

The module should assign the output port to the same value as the input
port combinationally.
```
</details>

#### reference
```verilog
module RefModule (
  input in,
  output out
);

  assign out = in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire in,
    output wire out
);
    assign out = in;
endmodule
```
---

### Prob009_popcount3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (3 bits)
 - output out (2 bits)

The module should implement a "population count" circuit that counts the
number of '1's in the input vector.
```
</details>

#### reference
```verilog
module RefModule (
  input [2:0] in,
  output [1:0] out
);

  assign out = in[0]+in[1]+in[2];

endmodule
```

#### model output
```verilog
module TopModule (
    input  [2:0] in,
    output [1:0] out
);
    // Population count of the 3‑bit input
    assign out = in[0] + in[1] + in[2];
endmodule
```
---

### Prob010_mt2015_q4a
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  x
 - input  y
 - output z

The module should implement the boolean function z = (x^y) & x.
```
</details>

#### reference
```verilog
module RefModule (
  input x,
  input y,
  output z
);

  assign z = (x^y) & x;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire x,
    input  wire y,
    output wire z
);
    assign z = (x ^ y) & x;
endmodule
```
---

### Prob011_norgate
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - output out

The module should implement a NOR gate.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  output out
);

  assign out = ~(a | b);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    output wire out
);
    assign out = ~(a | b);
endmodule
```
---

### Prob012_xnorgate
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - output out

The module should implement an XNOR gate.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  output out
);

  assign out = ~(a^b);

endmodule
```

#### model output
```verilog
module TopModule(
    input  a,
    input  b,
    output out
);
    assign out = ~(a ^ b);
endmodule
```
---

### Prob013_m2014_q4e
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in1
 - input  in2
 - output out

The module should implement a 2-input NOR gate.
```
</details>

#### reference
```verilog
module RefModule (
  input in1,
  input in2,
  output logic out
);

  assign out = ~(in1 | in2);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire in1,
    input  wire in2,
    output wire out
);
    assign out = ~(in1 | in2);
endmodule
```
---

### Prob014_andgate
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - output out

The module should implement a 2-input AND gate.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  output out
);

  assign out = a & b;

endmodule
```

#### model output
```verilog
module TopModule(
    input  a,
    input  b,
    output out
);
    assign out = a & b;
endmodule
```
---

### Prob015_vector1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in     (16 bits)
 - output out_hi ( 8 bits)
 - output out_lo ( 8 bits)

The module should implement a combinational circuit that splits an input
half-word (16 bits, [15:0] ) into lower [7:0] and upper [15:8] bytes.
```
</details>

#### reference
```verilog
module RefModule (
  input [15:0] in,
  output [7:0] out_hi,
  output [7:0] out_lo
);

  assign {out_hi, out_lo} = in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  [15:0] in,
    output [7:0]  out_hi,
    output [7:0]  out_lo
);
    assign out_hi = in[15:8];
    assign out_lo = in[7:0];
endmodule
```
---

### Prob016_m2014_q4j
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  x   (4 bits)
 - input  y   (4 bits)
 - output sum (5 bits)

Implement a 4-bit adder with full adders. The output sum should include
the overflow bit.
```
</details>

#### reference
```verilog
module RefModule (
  input [3:0] x,
  input [3:0] y,
  output [4:0] sum
);

  assign sum = x+y;

endmodule
```

#### model output
```verilog
module TopModule (
    input  [3:0] x,
    input  [3:0] y,
    output [4:0] sum
);
    wire c0, c1, c2, c3, c4;

    assign c0 = 1'b0;

    // Full adder instances
    full_adder fa0 (.a(x[0]), .b(y[0]), .cin(c0), .sum(sum[0]), .cout(c1));
    full_adder fa1 (.a(x[1]), .b(y[1]), .cin(c1), .sum(sum[1]), .cout(c2));
    full_adder fa2 (.a(x[2]), .b(y[2]), .cin(c2), .sum(sum[2]), .cout(c3));
    full_adder fa3 (.a(x[3]), .b(y[3]), .cin(c3), .sum(sum[3]), .cout(c4));

    assign sum[4] = c4;
endmodule

// 1‑bit full adder
module full_adder (
    input  a,
    input  b,
    input  cin,
    output sum,
    output cout
);
    assign {cout, sum} = a + b + cin;
endmodule
```
---

### Prob017_mux2to1v
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a   (100 bits)
 - input  b   (100 bits)
 - input  sel
 - output out (100 bits)

The module should implement a 2-1 multiplexer. When sel=0, choose a. When
sel=1, choose b.
```
</details>

#### reference
```verilog
module RefModule (
  input [99:0] a,
  input [99:0] b,
  input sel,
  output [99:0] out
);

  assign out = sel ? b : a;

endmodule
```

#### model output
```verilog
module TopModule (
    input  [99:0] a,
    input  [99:0] b,
    input         sel,
    output [99:0] out
);
    assign out = sel ? b : a;
endmodule
```
---

### Prob018_mux256to1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (256 bits)
 - input  sel (  8 bits)
 - output out

The module should implement a 1-bit wide, 256-to-1 multiplexer. The 256
inputs are all packed into a single 256-bit input vector. sel=0 should
select in[0], sel=1 selects bits in[1], sel=2 selects bits in[2], etc.
```
</details>

#### reference
```verilog
module RefModule (
  input [255:0] in,
  input [7:0] sel,
  output  out
);

  assign out = in[sel];

endmodule
```

#### model output
```verilog
module TopModule (
    input  [255:0] in,
    input  [7:0]   sel,
    output         out
);
    assign out = in[sel];
endmodule
```
---

### Prob019_m2014_q4f
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in1
 - input  in2
 - output out

The module should implement the following circuit in Verilog. Two inputs
(in1 and in2) go to an AND gate, but the in2 input to the AND gate has a
bubble. The output of the AND gate is connected to 'out'.
```
</details>

#### reference
```verilog
module RefModule (
  input in1,
  input in2,
  output logic out
);

  assign out = in1 & ~in2;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire in1,
    input  wire in2,
    output wire out
);
    assign out = in1 & ~in2;
endmodule
```
---

### Prob020_mt2015_eq2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  A (2 bits)
 - input  B (2 bits)
 - output z

The module should implement a circuit that has two 2-bit inputs A[1:0]
and B[1:0], and produces an output z. The value of z should be 1 if A =
B, otherwise z should be 0.
```
</details>

#### reference
```verilog
module RefModule (
  input [1:0] A,
  input [1:0] B,
  output z
);

  assign z = A[1:0]==B[1:0];

endmodule
```

#### model output
```verilog
module TopModule (
    input  [1:0] A,
    input  [1:0] B,
    output       z
);
    assign z = (A == B);
endmodule
```
---

### Prob021_mux256to1v
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (1024 bits)
 - input  sel (   8 bits)
 - output out (   4 bits)

The module should implement a 4-bit wide, 256-to-1 multiplexer. The 256
4-bit inputs are all packed into a single 1024-bit input vector. sel=0
should select bits in[3:0], sel=1 selects bits in[7:4], sel=2 selects
bits in[11:8], etc.
```
</details>

#### reference
```verilog
module RefModule (
  input [1023:0] in,
  input [7:0] sel,
  output [3:0] out
);

  assign out = {in[sel*4+3], in[sel*4+2], in[sel*4+1], in[sel*4+0]};

endmodule
```

#### model output
```verilog
module TopModule (
    input  [1023:0] in,
    input  [7:0]    sel,
    output [3:0]    out
);
    assign out = in[sel*4 +: 4];
endmodule
```
---

### Prob022_mux2to1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  sel
 - output out

The module should implement a one-bit wide, 2-to-1 multiplexer. When
sel=0, choose a. When sel=1, choose b.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input sel,
  output out
);

  assign out = sel ? b : a;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    input  wire sel,
    output wire out
);
    assign out = sel ? b : a;
endmodule
```
---

### Prob023_vector100r
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (100 bits)
 - output out (100 bits)

The module should reverse the bit ordering of the input and write to the
output.
```
</details>

#### reference
```verilog
module RefModule (
  input [99:0] in,
  output reg [99:0] out
);

  always_comb
    for (int i=0;i<$bits(out);i++)
      out[i] = in[$bits(out)-i-1];

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [99:0] in,
    output wire [99:0] out
);
    genvar i;
    generate
        for (i = 0; i < 100; i = i + 1) begin : rev
            assign out[i] = in[99-i];
        end
    endgenerate
endmodule
```
---

### Prob024_hadd
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - output sum
 - output cout

The module should implement a half adder. A half adder adds two bits
(with no carry-in) and produces a sum and carry-out.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  output sum,
  output cout
);

  assign {cout, sum} = a+b;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    output wire sum,
    output wire cout
);
    assign sum  = a ^ b; // XOR for sum
    assign cout = a & b; // AND for carry-out
endmodule
```
---

### Prob025_reduction
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in     (8 bits)
 - output parity

Parity checking is often used as a simple method of detecting errors when
transmitting data through an imperfect channel. The module should compute
a parity bit for an 8-bit byte (which will add a 9th bit to the byte). We
will use "even" parity, where the parity bit is just the XOR of all 8
data bits.
```
</details>

#### reference
```verilog
module RefModule (
  input [7:0] in,
  output parity
);

  assign parity = ^in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  [7:0] in,
    output       parity
);
    assign parity = ^in; // even parity (XOR of all bits)
endmodule
```
---

### Prob026_alwaysblock1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - output out_assign
 - output out_alwaysblock

The module should implement an AND gate using both an assign statement
and a combinational always block.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  output out_assign,
  output reg out_alwaysblock
);

  assign out_assign = a & b;
  always @(*) out_alwaysblock = a & b;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    output wire out_assign,
    output reg  out_alwaysblock
);

// AND gate using continuous assignment
assign out_assign = a & b;

// AND gate using combinational always block
always @* begin
    out_alwaysblock = a & b;
end

endmodule
```
---

### Prob027_fadd
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  cin
 - output cout
 - output sum

The module should impement a full adder. A full adder adds three bits
(including carry-in) and produces a sum and carry-out.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input cin,
  output cout,
  output sum
);

  assign {cout, sum} = a+b+cin;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    input  wire cin,
    output wire cout,
    output wire sum
);
    // Sum: XOR of all three inputs
    assign sum  = a ^ b ^ cin;
    // Carry-out: majority function of the three inputs
    assign cout = (a & b) | (b & cin) | (a & cin);
endmodule
```
---

### Prob028_m2014_q4a
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  d
 - input  ena
 - output q

The module should impement a D latch using an always block.
```
</details>

#### reference
```verilog
module RefModule (
  input d,
  input ena,
  output logic q
);

  always@(*) begin
    if (ena)
      q = d;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire d,
    input  wire ena,
    output reg  q
);
    // D latch: transparent when ena is high
    always @(*) begin
        if (ena)
            q = d;
        else
            q = q; // hold previous state
    end
endmodule
```
---

### Prob029_m2014_q4g
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in1
 - input  in2
 - input  in3
 - output out

The module should impement the following circuit: A two-input XNOR
(connected to 'in1' and 'in2) has an output connected to the input of a
two-input XOR. The second input of the XOR is 'in3.' The output of the
XOR is 'out'.
```
</details>

#### reference
```verilog
module RefModule (
  input in1,
  input in2,
  input in3,
  output logic out
);

  assign out = (~(in1 ^ in2)) ^ in3;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire in1,
    input  wire in2,
    input  wire in3,
    output wire out
);
    wire xnor_out;

    // Two‑input XNOR of in1 and in2
    assign xnor_out = ~(in1 ^ in2);

    // Two‑input XOR of the XNOR result and in3
    assign out = xnor_out ^ in3;
endmodule
```
---

### Prob030_popcount255
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (255 bits)
 - output out (  8 bits)

A "population count" circuit counts the number of '1's in an input
vector. The module should implement a population count circuit for a
255-bit input vector.
```
</details>

#### reference
```verilog
module RefModule (
  input [254:0] in,
  output reg [7:0] out
);

  always_comb  begin
    out = 0;
    for (int i=0;i<255;i++)
      out = out + in[i];
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [254:0] in,
    output reg  [7:0]   out
);
    integer i;
    always @* begin
        out = 8'd0;
        for (i = 0; i < 255; i = i + 1) begin
            out = out + in[i];
        end
    end
endmodule
```
---

### Prob031_dff
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input clk
 - input d
 - input q

The module should implement a single D flip-flop. Assume all sequential
logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input d,
  output reg q
);

  initial
    q = 1'hx;

  always @(posedge clk)
    q <= d;

endmodule
```

#### model output
```verilog
module TopModule (
    input  clk,
    input  d,
    output reg q
);
    always @(posedge clk) begin
        q <= d;
    end
endmodule
```
---

### Prob032_vector0
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  vec  (3 bits)
 - output outv (3 bits)
 - output o2
 - output o1
 - output o0

The module has one 3-bit input, then outputs the same vector, and also
splits it into three separate 1-bit outputs. Connect output o0 to the
input vector's position 0, o1 to position 1, etc.
```
</details>

#### reference
```verilog
module RefModule (
  input [2:0] vec,
  output [2:0] outv,
  output o2,
  output o1,
  output o0
);

  assign outv = vec;
  assign {o2, o1, o0} = vec;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [2:0] vec,
    output wire [2:0] outv,
    output wire       o2,
    output wire       o1,
    output wire       o0
);
    // Pass-through vector
    assign outv = vec;

    // Split vector bits to individual outputs
    assign o0 = vec[0];
    assign o1 = vec[1];
    assign o2 = vec[2];
endmodule
```
---

### Prob033_ece241_2014_q1c
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a (8 bits)
 - input  b (8 bits)
 - output s (8 bits)
 - output overflow

Assume that you have two 8-bit 2's complement numbers, a[7:0] and b[7:0].
The module should add these numbers to produce s[7:0]. Also compute
whether a (signed) overflow has occurred.
```
</details>

#### reference
```verilog
module RefModule (
  input [7:0] a,
  input [7:0] b,
  output [7:0] s,
  output overflow
);

  wire [8:0] sum = a+b;
  assign s = sum[7:0];
  assign overflow = !(a[7]^b[7]) && (a[7] != s[7]);

endmodule
```

#### model output
```verilog
module TopModule (
    input  [7:0] a,
    input  [7:0] b,
    output [7:0] s,
    output       overflow
);
    // Perform signed addition
    assign s = a + b;

    // Signed overflow detection: occurs when the sign bits of the
    // operands are the same and differ from the sign bit of the result.
    assign overflow = (a[7] == b[7]) && (s[7] != a[7]);

endmodule
```
---

### Prob034_dff8
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  d   (8 bits)
 - output q   (8 bits)

The module should include 8 D flip-flops. All DFFs should be triggered by
the positive edge of clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] d,
  output reg [7:0] q
);

  initial
    q = 8'h0;

  always @(posedge clk)
    q <= d;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire [7:0]  d,
    output reg  [7:0]  q
);

    // 8 D flip-flops, positive‑edge triggered
    always @(posedge clk) begin
        q <= d;
    end

endmodule
```
---

### Prob035_count1to10
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output q (4 bits)

The module should implement a decade counter that counts 1 through 10,
inclusive. Assume all sequential logic is triggered on the positive edge
of the clock. The reset input is active high synchronous, and should
reset the counter to 1.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output reg [3:0] q
);

  always @(posedge clk)
    if (reset || q == 10)
      q <= 1;
    else
      q <= q+1;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    output reg  [3:0] q
);

    // Sequential logic for decade counter (1 to 10)
    always @(posedge clk) begin
        if (reset) begin
            q <= 4'd1;               // Synchronous active‑high reset to 1
        end else if (q == 4'd10) begin
            q <= 4'd1;               // Wrap around after reaching 10
        end else begin
            q <= q + 4'd1;           // Increment
        end
    end

endmodule
```
---

### Prob036_ringer
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  ring
 - input  vibrate_mode
 - output ringer
 - output motor

The module should implement a circuit to control a cellphone's ringer and
vibration motor. Whenever the phone needs to ring from an incoming call
(input ring), your circuit must either turn on the ringer (output ringer
= 1) or the motor (output motor = 1), but not both. If the phone is in
vibrate mode (input vibrate_mode = 1), turn on the motor. Otherwise, turn
on the ringer.
```
</details>

#### reference
```verilog
module RefModule (
  input ring,
  input vibrate_mode,
  output ringer,
  output motor
);

  assign ringer = ring & ~vibrate_mode;
  assign motor = ring & vibrate_mode;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire ring,
    input  wire vibrate_mode,
    output wire ringer,
    output wire motor
);
    assign motor  = ring & vibrate_mode;
    assign ringer = ring & ~vibrate_mode;
endmodule
```
---

### Prob037_review2015_count1k
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output q (10 bits)

The module should implement a counter that counts from 0 to 999,
inclusive, with a period of 1000 cycles. Assume all sequential logic is
triggered on the positive edge of the clock. The reset input is active
high synchronous, and should reset the counter to 0.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output reg [9:0] q
);

  always @(posedge clk)
    if (reset || q == 999)
      q <= 0;
    else
      q <= q+1;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire       clk,
    input  wire       reset,
    output reg [9:0]  q
);
    // Synchronous counter: 0 to 999, then wrap
    always @(posedge clk) begin
        if (reset) begin
            q <= 10'd0;
        end else if (q == 10'd999) begin
            q <= 10'd0;
        end else begin
            q <= q + 10'd1;
        end
    end
endmodule
```
---

### Prob038_count15
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output q (4 bits)

The module should implement a 4-bit binary counter that counts from 0
through 15, inclusive, with a period of 16. Assume all sequential logic
is triggered on the positive edge of the clock. The reset input is active
high synchronous, and should reset the counter to 0.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output reg [3:0] q
);

  always @(posedge clk)
    if (reset)
      q <= 0;
    else
      q <= q+1;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    output reg  [3:0] q
);

    // Synchronous active‑high reset counter
    always @(posedge clk) begin
        if (reset)
            q <= 4'b0000;
        else
            q <= q + 1'b1;   // 4‑bit overflow wraps from 15 to 0
    end

endmodule
```
---

### Prob039_always_if
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  sel_b1
 - input  sel_b2
 - output out_assign
 - output out_always

The module should implement a 2-to-1 mux that chooses between a and b.
Choose b if both sel_b1 and sel_b2 are true. Otherwise, choose a. Do the
same twice, once using assign statements and once using a procedural if
statement.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input sel_b1,
  input sel_b2,
  output out_assign,
  output reg out_always
);

  assign out_assign = (sel_b1 & sel_b2) ? b : a;
  always @(*) out_always = (sel_b1 & sel_b2) ? b : a;

endmodule
```

#### model output
```verilog
module TopModule(
    input  wire a,
    input  wire b,
    input  wire sel_b1,
    input  wire sel_b2,
    output wire out_assign,
    output reg  out_always
);
    // Combinational assign implementation
    assign out_assign = (sel_b1 && sel_b2) ? b : a;

    // Procedural always block implementation
    always @* begin
        out_always = (sel_b1 && sel_b2) ? b : a;
    end
endmodule
```
---

### Prob040_count10
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output q (4 bits)

The module should implement a decade counter that counts from 0 through
9, inclusive, with a period of 10. Assume all sequential logic is
triggered on the positive edge of the clock. The reset input is active
high synchronous, and should reset the counter to 0.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output reg [3:0] q
);

  always @(posedge clk)
    if (reset || q == 9)
      q <= 0;
    else
      q <= q+1;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    output reg  [3:0] q
);

    // Internal counter register
    reg [3:0] count;

    // Sequential logic: synchronous active‑high reset
    always @(posedge clk) begin
        if (reset) begin
            count <= 4'd0;
        end else if (count == 4'd9) begin
            count <= 4'd0;
        end else begin
            count <= count + 4'd1;
        end
    end

    // Assign output
    always @(*) begin
        q = count;
    end

endmodule
```
---

### Prob041_dff8r
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  d (8 bits)
 - output q (8 bits)

The module should include 8 D flip-flops with active high synchronous
reset setting the output to zero. All DFFs should be triggered by the
positive edge of clk.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] d,
  input reset,
  output reg [7:0] q
);

  always @(posedge clk)
    if (reset)
      q <= 0;
    else
      q <= d;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        reset,
    input  wire [7:0]  d,
    output reg  [7:0]  q
);
    always @(posedge clk) begin
        if (reset)
            q <= 8'b0;
        else
            q <= d;
    end
endmodule
```
---

### Prob042_vector4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (8  bits)
 - output out (32 bits)

One common place to see a replication operator is when sign-extending a
smaller number to a larger one, while preserving its signed value. This
is done by replicating the sign bit (the most significant bit) of the
smaller number to the left. For example, sign-extending 4'b0101 (5) to 8
bits results in 8'b00000101 (5), while sign-extending 4'b1101 (-3) to 8
bits results in 8'b11111101 (-3). Implement a module that sign-extends an
8-bit number to 32 bits. This requires a concatenation of 24 copies of
the sign bit (i.e., replicate bit[7] 24 times) followed by the 8-bit
number itself.
```
</details>

#### reference
```verilog
module RefModule (
  input [7:0] in,
  output [31:0] out
);

  assign out = { {24{in[7]}}, in };

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [7:0]  in,
    output wire [31:0] out
);
    assign out = {{24{in[7]}}, in};
endmodule
```
---

### Prob043_vector5
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - input  e
 - output out (25 bits)

Implement a module that given five 1-bit signals (a, b, c, d, and e),
compute all 25 pairwise one-bit comparisons in the 25-bit output vector.
The output should be 1 if the two bits being compared are equal. Example:
out[24] = ~a ^ a; out[23] = ~a ^ b; out[22] = ~a ^ c; ...
out[ 1] = ~e ^ d; out[ 0] = ~e ^ e.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  input e,
  output [24:0] out
);

  assign out = ~{ {5{a}}, {5{b}}, {5{c}}, {5{d}}, {5{e}} } ^ {5{a,b,c,d,e}};

endmodule
```

#### model output
```verilog

```
---

### Prob044_vectorgates
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a              (3 bits)
 - input  b              (3 bits)
 - output out_or_bitwise (3 bits)
 - output out_or_logical
 - output out_not        (6 bits)

Implement a module with two 3-bit inputs that computes the bitwise-OR of
the two vectors, the logical-OR of the two vectors, and the inverse (NOT)
of both vectors. Place the inverse of b in the upper half of out_not
(i.e., bits [5:3]), and the inverse of a in the lower half.
```
</details>

#### reference
```verilog
module RefModule (
  input [2:0] a,
  input [2:0] b,
  output [2:0] out_or_bitwise,
  output out_or_logical,
  output [5:0] out_not
);

  assign out_or_bitwise = a | b;
  assign out_or_logical = a || b;
  assign out_not = {~b,~a};

endmodule
```

#### model output
```verilog
module TopModule (
    input  [2:0] a,
    input  [2:0] b,
    output [2:0] out_or_bitwise,
    output       out_or_logical,
    output [5:0] out_not
);
    // Bitwise OR of the two 3‑bit vectors
    assign out_or_bitwise = a | b;

    // Logical OR of the two vectors (true if any bit of a or b is 1)
    assign out_or_logical = |a | |b;

    // Inverse (bitwise NOT) of b in the upper half, a in the lower half
    assign out_not = {~b, ~a};
endmodule
```
---

### Prob045_edgedetect2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input clk
 - input in       (8 bits)
 - output anyedge (8 bits)

Implement a module that for each bit in an 8-bit input vector, detect
when the input signal changes from one clock cycle to the next (detect
any edge). The output bit of anyedge should be set to 1 the cycle 
after the input bit has 0 to 1 or 1 to 0 transition occurs. Assume all 
sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] in,
  output reg [7:0] anyedge
);

  reg [7:0] d_last;

  always @(posedge clk) begin
    d_last <= in;
    anyedge <= in ^ d_last;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input        clk,
    input  [7:0] in,
    output reg [7:0] anyedge
);
    // Register to hold the input value from the previous clock cycle
    reg [7:0] prev_in;

    // Edge detection: anyedge is asserted for one cycle after a change
    always @(posedge clk) begin
        anyedge <= in ^ prev_in; // 1 if current bit differs from previous bit
        prev_in <= in;           // Update previous value for next cycle
    end
endmodule
```
---

### Prob046_dff8p
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  d (8 bits)
 - output q (8 bits)

Implement a module that includes 8 D flip-flops with active high
synchronous reset. The flip-flops must be reset to 0x34 rather than zero.
All DFFs should be triggered by the negative edge of clk.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] d,
  input reset,
  output reg [7:0] q
);

  always @(negedge clk)
    if (reset)
      q <= 8'h34;
    else
      q <= d;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        reset,
    input  wire [7:0]  d,
    output reg  [7:0]  q
);

    // 8-bit register implemented as 8 D flip‑flops
    // Negative‑edge triggered, synchronous active‑high reset to 0x34
    always @(negedge clk) begin
        if (reset)
            q <= 8'h34;
        else
            q <= d;
    end

endmodule
```
---

### Prob047_dff8ar
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  d      (8 bits)
 - output q      (8 bits)

The module should include 8 D flip-flops with active high asynchronous
reset. The output should be reset to 0. All DFFs should be triggered by
the positive edge of clk.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] d,
  input areset,
  output reg [7:0] q
);

  always @(posedge clk, posedge areset)
    if (areset)
      q <= 0;
    else
      q <= d;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        areset,
    input  wire [7:0]  d,
    output reg  [7:0]  q
);

    // 8 D flip‑flops with active‑high asynchronous reset
    always @(posedge clk or posedge areset) begin
        if (areset) begin
            q <= 8'b0;          // reset output to 0
        end else begin
            q <= d;             // capture input data
        end
    end

endmodule
```
---

### Prob048_m2014_q4c
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  d
 - input  r
 - output q

The module should implement a simple D flip flop with active high
synchronous reset (reset output to 0).
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input d,
  input r,
  output logic q
);

  always@(posedge clk) begin
    if (r)
      q <= 0;
    else
      q <= d;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  clk,
    input  d,
    input  r,
    output reg q
);
    always @(posedge clk) begin
        if (r)
            q <= 1'b0;
        else
            q <= d;
    end
endmodule
```
---

### Prob049_m2014_q4b
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  ar
 - input  d
 - output q

The module should implement a D flip flop, positive edge triggered, with
an asynchronous reset "ar".
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input d,
  input ar,
  output logic q
);

  always@(posedge clk or posedge ar) begin
    if (ar)
      q <= 0;
    else
      q <= d;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire ar,
    input  wire d,
    output reg  q
);
    always @(posedge clk or posedge ar) begin
        if (ar)
            q <= 1'b0;
        else
            q <= d;
    end
endmodule
```
---

### Prob050_kmap1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - output out

The module should implement the circuit described by the Karnaugh map
below.

          a
   bc   0   1
   00 | 0 | 1 |
   01 | 1 | 1 |
   11 | 1 | 1 |
   10 | 1 | 1 |
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  output out
);

  assign out = (a | b | c);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    input  wire c,
    output wire out
);
    // Output is 1 for all combinations except a=0, b=0, c=0
    assign out = a | b | c;
endmodule
```
---

### Prob051_gates4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in (4 bits)
 - output out_and
 - output out_or
 - output out_xor

The module should implement a combinational circuit with four inputs,
in[3:0]. There are 3 outputs:

  (1) out_and : output of a 4-input AND gate
  (2) out_or  : output of a 4-input OR  gate
  (3) out_xor : output of a 4-input XOR gate
```
</details>

#### reference
```verilog
module RefModule (
  input [3:0] in,
  output out_and,
  output out_or,
  output out_xor
);

  assign out_and = &in;
  assign out_or = |in;
  assign out_xor = ^in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  [3:0] in,
    output       out_and,
    output       out_or,
    output       out_xor
);
    // 4‑input AND
    assign out_and = &in;
    // 4‑input OR
    assign out_or  = |in;
    // 4‑input XOR
    assign out_xor = ^in;
endmodule
```
---

### Prob052_gates100
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in (100 bits)
 - output out_and
 - output out_or
 - output out_xor

The module should implement a combinational circuit with 100 inputs,
in[99:0]. There are 3 outputs:

  (1) out_and : output of a 100-input AND gate
  (2) out_or  : output of a 100-input OR  gate
  (3) out_xor : output of a 100-input XOR gate
```
</details>

#### reference
```verilog
module RefModule (
  input [99:0] in,
  output out_and,
  output out_or,
  output out_xor
);

  assign out_and = &in;
  assign out_or = |in;
  assign out_xor = ^in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [99:0] in,
    output wire        out_and,
    output wire        out_or,
    output wire        out_xor
);
    // 100-input AND
    assign out_and = &in;
    // 100-input OR
    assign out_or  = |in;
    // 100-input XOR
    assign out_xor = ^in;
endmodule
```
---

### Prob053_m2014_q4d
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  in
 - output out

The module should implement the following circuit: A D flip-flop takes as
input the output of a two-input XOR. The flip-flop is positive edge
triggered by clk, but there is no reset. The XOR takes as input 'in'
along with the output 'out' of the flip-flop.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input in,
  output logic out
);

  initial
    out = 0;

  always@(posedge clk) begin
    out <= in ^ out;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  clk,
    input  in,
    output reg out
);
    always @(posedge clk) begin
        out <= in ^ out;
    end
endmodule
```
---

### Prob054_edgedetect
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  in    (8 bits)
 - output pedge (8 bits)

The module should examine each bit in an 8-bit vector and detect when the
input signal changes from 0 in one clock cycle to 1 the next (similar to
positive edge detection). The output bit should be set the cycle after a
0 to 1 transition occurs.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] in,
  output reg [7:0] pedge
);

  reg [7:0] d_last;

  always @(posedge clk) begin
    d_last <= in;
    pedge <= in & ~d_last;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire [7:0]  in,
    output reg  [7:0]  pedge
);

    // Register to hold previous value of 'in'
    reg [7:0] in_d1;
    // Register to hold detected 0→1 transitions (one‑cycle delayed)
    reg [7:0] trans;

    // Capture previous input value
    always @(posedge clk) begin
        in_d1 <= in;
    end

    // Detect 0→1 transitions (current cycle)
    always @(posedge clk) begin
        trans <= (~in_d1) & in;
    end

    // Output the transition detection one cycle later
    always @(posedge clk) begin
        pedge <= trans;
    end

endmodule
```
---

### Prob055_conditional
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a   (8 bits)
 - input  b   (8 bits)
 - input  c   (8 bits)
 - input  d   (8 bits)
 - output min (8 bits)

The module should find the minimum of the four input values. Unsigned
numbers can be compared with standard comparison operators (a < b).
```
</details>

#### reference
```verilog
module RefModule (
  input [7:0] a,
  input [7:0] b,
  input [7:0] c,
  input [7:0] d,
  output reg [7:0] min
);

  always_comb begin
    min = a;
    if (min > b) min = b;
    if (min > c) min = c;
    if (min > d) min = d;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  [7:0] a,
    input  [7:0] b,
    input  [7:0] c,
    input  [7:0] d,
    output reg [7:0] min
);
    always @* begin
        min = a;
        if (b < min) min = b;
        if (c < min) min = c;
        if (d < min) min = d;
    end
endmodule
```
---

### Prob056_ece241_2013_q7
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  j
 - input  k
 - output Q

The module should implement a JK flip-flop with the following truth
table. Note: Qold is the output of the flip-flop before the positive
clock edge.

  J | K | Q
  0 | 0 | Qold
  0 | 1 | 0
  1 | 0 | 1
  1 | 1 | ~Qold
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input j,
  input k,
  output reg Q
);

  always @(posedge clk)
    Q <= j&~Q | ~k&Q;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire j,
    input  wire k,
    output reg  Q
);

always @(posedge clk) begin
    case ({j, k})
        2'b00: Q <= Q;        // retain previous state
        2'b01: Q <= 1'b0;     // reset
        2'b10: Q <= 1'b1;     // set
        2'b11: Q <= Q;        // retain previous state
        default: Q <= Q;
    endcase
end

endmodule
```
---

### Prob057_kmap2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - output out

The module should implement the circuit described by the Karnaugh map
below.

              ab
   cd   00  01  11  10
   00 | 1 | 1 | 0 | 1 |
   01 | 1 | 0 | 0 | 1 |
   11 | 0 | 1 | 1 | 1 |
   10 | 1 | 1 | 0 | 0 |
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output out
);

  assign out = (~c & ~b) | (~d&~a) | (a&c&d) | (b&c&d);

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob058_alwaysblock2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  a
 - input  b
 - output out_assign
 - output out_always_comb
 - output out_always_ff

The module should implement an XOR gate three ways, using an assign
statement (output out_assign), a combinational always block (output
out_always_comb), and a clocked always block (output out_always_ff). Note
that the clocked always block produces a different circuit from the other
two: There is a flip- flop so the output is delayed. Assume all
sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input a,
  input b,
  output out_assign,
  output reg out_always_comb,
  output reg out_always_ff
);

  assign out_assign = a ^ b;
  always @(*) out_always_comb = a ^ b;
  always @(posedge clk) out_always_ff <= a ^ b;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire a,
    input  wire b,
    output wire out_assign,
    output reg  out_always_comb,
    output reg  out_always_ff
);

    // XOR using continuous assignment
    assign out_assign = a ^ b;

    // XOR using combinational always block
    always @* begin
        out_always_comb = a ^ b;
    end

    // XOR using sequential always block (flip‑flop)
    always @(posedge clk) begin
        out_always_ff <= a ^ b;
    end

endmodule
```
---

### Prob059_wire4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - output w
 - output x
 - output y
 - output z

The module should behave like wires that makes these connections:

  a -> w
  b -> x
  b -> y
  c -> z
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  output w,
  output x,
  output y,
  output z
);

  assign {w,x,y,z} = {a,b,b,c};

endmodule
```

#### model output
```verilog
module TopModule (
    input  a,
    input  b,
    input  c,
    output w,
    output x,
    output y,
    output z
);
    assign w = a;
    assign x = b;
    assign y = b;
    assign z = c;
endmodule
```
---

### Prob060_m2014_q4k
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  resetn
 - input  in
 - output out

The module should implement a shift register with four D flops. Assume
all sequential logic is triggered on the positive edge of the clock.
Reset is active-low synchronous resettable.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input resetn,
  input in,
  output out
);

  reg [3:0] sr;
  always @(posedge clk) begin
    if (~resetn)
      sr <= '0;
    else
      sr <= {sr[2:0], in};
  end

  assign out = sr[3];

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire resetn,
    input  wire in,
    output wire out
);

    // 4-bit shift register
    reg [3:0] shift_reg;

    // Sequential logic with synchronous active-low reset
    always @(posedge clk) begin
        if (!resetn)
            shift_reg <= 4'b0000;
        else
            shift_reg <= {shift_reg[2:0], in};
    end

    // Output is the MSB of the shift register
    assign out = shift_reg[3];

endmodule
```
---

### Prob061_2014_q4a
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  w
 - input  R
 - input  E
 - input  L
 - output Q

The module will be one stage in a larger n-bit shift register circuit.
Input E is for enabling shift, R for value to load, L is asserted when it
should load, and w is the input from the prevous stage of the shift
register. Assume all sequential logic is triggered on the positive edge
of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input w,
  input R,
  input E,
  input L,
  output reg Q
);

  always @(posedge clk)
    if (L)
      Q <= R;
    else if (E)
      Q <= w;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire w,
    input  wire R,
    input  wire E,
    input  wire L,
    output reg  Q
);
    always @(posedge clk) begin
        if (L)
            Q <= R;
        else if (E)
            Q <= w;
        else
            Q <= Q;
    end
endmodule
```
---

### Prob062_bugs_mux2
<details>
<summary>prompt</summary>

```text
Consider the following implementation of an 8-bit 2-to-1 mux:

  module TopModule (
      input        sel,
      input  [7:0] a,
      input  [7:0] b,
      output       out
  );

      assign out = (~sel & a) | (sel & b);

  endmodule

Unfortunately, this module has a bug. Implement a new version of this
module that fixes the bug.
```
</details>

#### reference
```verilog
module RefModule (
  input sel,
  input [7:0] a,
  input [7:0] b,
  output reg [7:0] out
);

  // assign out = (~sel & a) | (sel & b);
  assign out = sel ? a : b;

endmodule
```

#### model output
```verilog
module TopModule (
    input        sel,
    input  [7:0] a,
    input  [7:0] b,
    output [7:0] out
);

    // Correct 2-to-1 mux: when sel=0 output a, when sel=1 output b
    assign out = (sel) ? b : a;
    // Alternatively:
    // assign out = (sel & b) | (~sel & a);

endmodule
```
---

### Prob063_review2015_shiftcount
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  shift_ena
 - input  count_ena
 - input  data
 - output q (4 bits)

The module should implement a four-bit shift register that also acts as a
down counter. Data is shifted in most-significant-bit first when
shift_ena is 1. The number currently in the shift register is decremented
when count_ena is 1. Since the full system doesn't ever use shift_ena and
count_ena together, it does not matter what your circuit does if both
control inputs are 1 (this mainly means that it doesn't matter which case
gets higher priority). Assume all sequential logic is triggered on the
positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input shift_ena,
  input count_ena,
  input data,
  output reg [3:0] q
);

  always @(posedge clk) begin
    if (shift_ena)
      q <= { q[2:0], data };
    else if (count_ena)
      q <= q - 1'b1;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire shift_ena,
    input  wire count_ena,
    input  wire data,
    output reg  [3:0] q
);

always @(posedge clk) begin
    if (shift_ena) begin
        // Shift in data MSB‑first
        q <= {data, q[3:1]};
    end else if (count_ena) begin
        // Decrement the 4‑bit value
        q <= q - 4'd1;
    end
    // If neither enable is asserted, retain current value (implicit)
end

endmodule
```
---

### Prob064_vector3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a (5 bits)
 - input  b (5 bits)
 - input  c (5 bits)
 - input  d (5 bits)
 - input  e (5 bits)
 - input  f (5 bits)
 - output w (8 bits)
 - output x (8 bits)
 - output y (8 bits)
 - output z (8 bits)

The module should concatenate the input vectors together then split them
up into several output vectors. There are six 5-bit input vectors: a, b,
c, d, e, and f, for a total of 30 bits of input. There are four 8-bit
output vectors: w, x, y, and z, for 32 bits of output. The output should
be a concatenation of the input vectors followed by two 1 bits (the two 1
bits should be in the LSB positions).
```
</details>

#### reference
```verilog
module RefModule (
  input [4:0] a,
  input [4:0] b,
  input [4:0] c,
  input [4:0] d,
  input [4:0] e,
  input [4:0] f,
  output [7:0] w,
  output [7:0] x,
  output [7:0] y,
  output [7:0] z
);

  assign { w,x,y,z} = {a,b,c,d,e,f,2'b11};

endmodule
```

#### model output
```verilog
module TopModule (
    input  [4:0] a,
    input  [4:0] b,
    input  [4:0] c,
    input  [4:0] d,
    input  [4:0] e,
    input  [4:0] f,
    output [7:0] w,
    output [7:0] x,
    output [7:0] y,
    output [7:0] z
);
    // Concatenate inputs and two trailing '1' bits
    wire [31:0] concat_vec = {a, b, c, d, e, f, 2'b11};

    // Split the 32‑bit vector into four 8‑bit outputs
    assign w = concat_vec[31:24];
    assign x = concat_vec[23:16];
    assign y = concat_vec[15:8];
    assign z = concat_vec[7:0];
endmodule
```
---

### Prob065_7420
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  p1a
 - input  p1b
 - input  p1c
 - input  p1d
 - input  p2a
 - input  p2b
 - input  p2c
 - input  p2d
 - output p1y
 - output p2y

The 7400-series integrated circuits are a series of digital chips with a
few gates each. The 7420 is a chip with two 4-input NAND gates. The
module should implement the same functionality as the 7420 chip.
```
</details>

#### reference
```verilog
module RefModule (
  input p1a,
  input p1b,
  input p1c,
  input p1d,
  output p1y,
  input p2a,
  input p2b,
  input p2c,
  input p2d,
  output p2y
);

  assign p1y = ~&( {p1a, p1b, p1c, p1d} );
  assign p2y = ~&( {p2a, p2b, p2c, p2d} );

endmodule
```

#### model output
```verilog
module TopModule (
    input  p1a,
    input  p1b,
    input  p1c,
    input  p1d,
    input  p2a,
    input  p2b,
    input  p2c,
    input  p2d,
    output p1y,
    output p2y
);
    // 7420: dual 4‑input NAND gates
    assign p1y = ~(p1a & p1b & p1c & p1d);
    assign p2y = ~(p2a & p2b & p2c & p2d);
endmodule
```
---

### Prob066_edgecapture
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  in    (32 bits)
 - output out   (32 bits)

The module should examine each bit in the 32-bit input vector, and
capture when the input signal changes from 1 in one clock cycle to 0 the
next. "Capture" means that the output will remain 1 until the register is
reset (active high synchronous reset). Assume all sequential logic is
triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input [31:0] in,
  output reg [31:0] out
);

  reg [31:0] d_last;

  always @(posedge clk) begin
    d_last <= in;
    if (reset)
      out <= '0;
    else
      out <= out | (~in & d_last);
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] in,
    output reg  [31:0] out
);

    // Register to hold previous value of 'in'
    reg [31:0] prev_in;

    // Sequential logic: synchronous reset, capture falling edges
    always @(posedge clk) begin
        if (reset) begin
            out     <= 32'b0;
            prev_in <= 32'b0;
        end else begin
            // Set output bits where a falling edge is detected
            out     <= out | (prev_in & ~in);
            // Update previous input
            prev_in <= in;
        end
    end

endmodule
```
---

### Prob067_countslow
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  slowena
 - output q (4 bits)

The module should implement a decade counter that counts from 0 through
9, inclusive, with a period of 10. The reset input is active high
synchronous, and should reset the counter to 0. We want to be able to
pause the counter rather than always incrementing every clock cycle, so
the "slowena" input if high indicates when the counter should increment.
Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input slowena,
  input reset,
  output reg [3:0] q
);

  always @(posedge clk)
    if (reset)
      q <= 0;
    else if (slowena) begin
      if (q == 9)
        q <= 0;
      else
        q <= q+1;
    end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire slowena,
    output reg  [3:0] q
);
    always @(posedge clk) begin
        if (reset) begin
            q <= 4'd0;
        end else if (slowena) begin
            if (q == 4'd9)
                q <= 4'd0;
            else
                q <= q + 4'd1;
        end
    end
endmodule
```
---

### Prob068_countbcd
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output ena (3 bits)
 - output q   (16 bits)

The module should implement a 4-digit BCD (binary-coded decimal) counter.
Each decimal digit is encoded using 4 bits: q[3:0] is the ones digit,
q[7:4] is the tens digit, etc. For digits [3:1], also output an enable
signal indicating when each of the upper three digits should be
incremented. Include a synchronous active-high reset. Assume all
sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output [3:1] ena,
  output reg [15:0] q
);

  wire [3:0] enable = { q[11:0]==12'h999, q[7:0]==8'h99, q[3:0] == 4'h9, 1'b1};
  assign ena = enable[3:1];
  always @(posedge clk)
    for (int i=0;i<4;i++) begin
      if (reset || (q[i*4 +:4] == 9 && enable[i]))
        q[i*4 +:4] <= 0;
      else if (enable[i])
        q[i*4 +:4] <= q[i*4 +:4]+1;
    end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob069_truthtable1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  x3
 - input  x2
 - input  x1
 - output f

The module should implement a combinational circuit for the following
truth table:

  x3 | x2 | x1 | f
  0  | 0  | 0  | 0
  0  | 0  | 1  | 0
  0  | 1  | 0  | 1
  0  | 1  | 1  | 1
  1  | 0  | 0  | 0
  1  | 0  | 1  | 1
  1  | 1  | 0  | 0
  1  | 1  | 1  | 1
```
</details>

#### reference
```verilog
module RefModule (
  input x3,
  input x2,
  input x1,
  output f
);

  assign f = ( ~x3 & x2 & ~x1 ) |
        ( ~x3 & x2 & x1 ) |
        ( x3 & ~x2 & x1 ) |
        ( x3 & x2 & x1 ) ;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire x3,
    input  wire x2,
    input  wire x1,
    output wire f
);
    assign f = (~x3 & x2) | (x3 & x1);
endmodule
```
---

### Prob070_ece241_2013_q2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

  - input  a
  - input  b
  - input  c
  - input  d
  - output out_sop
  - output out_pos

The module should implement a digital system with four inputs (a,b,c,d)
that generates a logic-1 when 2, 7, or 15 appears on the inputs, and a
logic-0 when 0, 1, 4, 5, 6, 9, 10, 13, or 14 appears. The input
conditions for the numbers 3, 8, 11, and 12 never occur in this system.
For example, 7 corresponds to a,b,c,d being set to 0,1,1,1, respectively.
Determine the output out_sop in minimum sum-of-products form, and the
output out_pos in minimum product-of-sums form.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output out_sop,
  output out_pos
);

  wire pos0, pos1;
  assign out_sop = c&d | ~a&~b&c;
  assign pos0 = c & (~b|d)&(~a|b);
  assign pos1 = c & (~b|d)&(~a|d);

  assign out_pos = (pos0 == pos1) ? pos0 : 1'bx;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob071_always_casez
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (8 bits)
 - output pos (3 bits)

The module should implement a priority encoder for an 8-bit input. Given
an 8-bit vector, the output should report the first (least significant)
bit in the vector that is 1. Report zero if the input vector has no bits
that are high. For example, the input 8'b10010000 should output 3'd4,
because bit[4] is first bit that is high.
```
</details>

#### reference
```verilog
module RefModule (
  input [7:0] in,
  output reg [2:0] pos
);

  always @(*) begin
    casez (in)
      default : pos = 2'h0;
      8'bzzzzzzz1: pos = 3'h0;
      8'bzzzzzz1z: pos = 3'h1;
      8'bzzzzz1zz: pos = 3'h2;
      8'bzzzz1zzz: pos = 3'h3;
      8'bzzz1zzzz: pos = 3'h4;
      8'bzz1zzzzz: pos = 3'h5;
      8'bz1zzzzzz: pos = 3'h6;
      8'b1zzzzzzz: pos = 3'h7;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [7:0] in,
    output reg  [2:0] pos
);

always @* begin
    casez (in)
        8'b???????1: pos = 3'd0;
        8'b??????10: pos = 3'd1;
        8'b?????100: pos = 3'd2;
        8'b????1000: pos = 3'd3;
        8'b???10000: pos = 3'd4;
        8'b??100000: pos = 3'd5;
        8'b?1000000: pos = 3'd6;
        8'b10000000: pos = 3'd7;
        default:     pos = 3'd0; // no bits set
    endcase
end

endmodule
```
---

### Prob072_thermostat
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  mode
 - input  too_cold
 - input  too_hot
 - input  fan_on
 - output heater
 - output aircon
 - output fan

The module should implement a heating/cooling thermostat controller which
controls both a heater (during winter) and an air conditioner (during
summer). The module should on and off the heater, air conditioning, and
blower fan as appropriate. The thermostat can be in one of two modes:
heating (mode = 1) and cooling (mode = 0). In heating mode, turn the
heater on when it is too cold (too_cold = 1) but do not use the air
conditioner. In cooling mode, turn the air conditioner on when it is too
hot (too_hot = 1), but do not turn on the heater. When the heater or air
conditioner are on, also turn on the fan to circulate the air. In
addition, the user can also request the fan to turn on (fan_on = 1), even
if the heater and air conditioner are off.
```
</details>

#### reference
```verilog
module RefModule (
  input mode,
  input too_cold,
  input too_hot,
  input fan_on,
  output heater,
  output aircon,
  output fan
);

  assign fan = (mode ? too_cold : too_hot) | fan_on;
  assign heater = (mode & too_cold);
  assign aircon = (~mode & too_hot);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire mode,       // 1 = heating, 0 = cooling
    input  wire too_cold,
    input  wire too_hot,
    input  wire fan_on,
    output wire heater,
    output wire aircon,
    output wire fan
);

    // Heater is active only in heating mode when it's too cold
    assign heater = mode & too_cold;

    // Air conditioner is active only in cooling mode when it's too hot
    assign aircon = (~mode) & too_hot;

    // Fan runs when either heater or aircon is on, or when the user requests it
    assign fan = fan_on | heater | aircon;

endmodule
```
---

### Prob073_dff16e
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  resetn
 - input  byteena ( 2 bits)
 - input  d       (16 bits)
 - output q       (16 bits)

The module should include 16 D flip-flops. It's sometimes useful to only
modify parts of a group of flip-flops. The byte-enable inputs control
whether each byte of the 16 registers should be written to on that cycle.
byteena[1] controls the upper byte d[15:8], while byteena[0] controls the
lower byte d[7:0]. resetn is a synchronous, active-low reset. All DFFs
should be triggered by the positive edge of clk.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input resetn,
  input [1:0] byteena,
  input [15:0] d,
  output reg [15:0] q
);

  always @(posedge clk) begin
    if (!resetn)
      q <= 0;
    else begin
      if (byteena[0])
        q[7:0] <= d[7:0];
      if (byteena[1])
        q[15:8] <= d[15:8];
    end
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        resetn,
    input  wire [1:0]  byteena,
    input  wire [15:0] d,
    output reg  [15:0] q
);

    // 16 D flip-flops with byte-enable write control and synchronous active‑low reset
    always @(posedge clk) begin
        if (!resetn) begin
            q <= 16'b0;
        end else begin
            if (byteena[1])
                q[15:8] <= d[15:8];
            if (byteena[0])
                q[7:0]  <= d[7:0];
        end
    end

endmodule
```
---

### Prob074_ece241_2014_q4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  x
 - output z

The module should implement a finite state machine with the following
behavior. Input x goes to three different two-input gates: an XOR, an
AND, and a OR gate. Each of the three gates is connected to the input of
a D flip-flop and then the flip-flop outputs all go to a three-input
NOR gate, whose output is Z. The second input of the XOR is its corresponding
flip-flop's output, the second input of the AND is its corresponding
flip-flop's complemented output, and finally the second input of the OR
is its corresponding flip-flop's complementary output.

Assume that the D flip-flops are initially reset to zero before the
machine begins. Assume all sequential logic is triggered on the positive
edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input x,
  output z
);

  reg [2:0] s = 0;

  always @(posedge clk) begin
    s <= { s[2] ^ x, ~s[1] & x, ~s[0] | x };
  end

  assign z = ~|s;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire x,
    output wire z
);

    // State registers
    reg q0;
    reg q1;
    reg q2;

    // Sequential logic: D flip-flops
    always @(posedge clk) begin
        q0 <= x ^ q0;          // XOR gate feedback
        q1 <= x & ~q1;         // AND gate with complemented feedback
        q2 <= x | ~q2;         // OR gate with complemented feedback
    end

    // Combinational logic: three‑input NOR producing Z
    assign z = ~(q0 | q1 | q2);

    // Optional: initialize flip‑flops to zero at simulation start
    initial begin
        q0 = 1'b0;
        q1 = 1'b0;
        q2 = 1'b0;
    end

endmodule
```
---

### Prob075_counter_2bc
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  train_valid
 - input  train_taken
 - output state (2 bits)

The module should implement a two-bit saturating counter. The counter
increments (up to a maximum of 3) when train_valid = 1 and
train_taken = 1. It decrements (down to a minimum of 0) when
train_valid = 1 and train_taken = 0. When not training (train_valid = 0),
the counter keeps its value unchanged. areset is a positive edge
triggered asynchronous reset that resets the counter to weakly not-taken
(2'b01). Output state[1:0] is the two-bit counter value. Assume all
sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input train_valid,
  input train_taken,
  output logic [1:0] state
);

  always @(posedge clk, posedge areset) begin
    if (areset)
      state <= 1;
    else if (train_valid) begin
      if(state < 3 && train_taken)
        state <= state + 1;
      else if(state > 0 && !train_taken)
        state <= state - 1;
    end
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire       clk,
    input  wire       areset,
    input  wire       train_valid,
    input  wire       train_taken,
    output reg  [1:0] state
);

    // Asynchronous positive‑edge reset, synchronous saturating counter
    always @(posedge clk or posedge areset) begin
        if (areset) begin
            state <= 2'b01;               // weakly not‑taken
        end else if (train_valid) begin
            if (train_taken) begin
                // Increment with saturation at 3
                if (state != 2'b11)
                    state <= state + 2'b01;
                else
                    state <= state;
            end else begin
                // Decrement with saturation at 0
                if (state != 2'b00)
                    state <= state - 2'b01;
                else
                    state <= state;
            end
        end else begin
            state <= state;                // hold value when not training
        end
    end

endmodule
```
---

### Prob076_always_case
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  sel   (3 bits)
 - input  data0 (4 bits)
 - input  data1 (4 bits)
 - input  data2 (4 bits)
 - input  data3 (4 bits)
 - input  data4 (4 bits)
 - input  data5 (4 bits)
 - output out   (4 bits)
);

The module should implement a 6-to-1 multiplexer. When sel is between 0
and 5, choose the corresponding data input. Otherwise, output 0. The data
inputs and outputs are all 4 bits wide.
```
</details>

#### reference
```verilog
module RefModule (
  input [2:0] sel,
  input [3:0] data0,
  input [3:0] data1,
  input [3:0] data2,
  input [3:0] data3,
  input [3:0] data4,
  input [3:0] data5,
  output reg [3:0] out
);

  always @(*) begin
    case (sel)
      3'h0: out = data0;
      3'h1: out = data1;
      3'h2: out = data2;
      3'h3: out = data3;
      3'h4: out = data4;
      3'h5: out = data5;
      default: out = 4'b0;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  [2:0] sel,
    input  [3:0] data0,
    input  [3:0] data1,
    input  [3:0] data2,
    input  [3:0] data3,
    input  [3:0] data4,
    input  [3:0] data5,
    output reg [3:0] out
);
    always @* begin
        case (sel)
            3'd0: out = data0;
            3'd1: out = data1;
            3'd2: out = data2;
            3'd3: out = data3;
            3'd4: out = data4;
            3'd5: out = data5;
            default: out = 4'b0;
        endcase
    end
endmodule
```
---

### Prob077_wire_decl
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - output out
 - output out_n

The module should implement the following circuit. Create two
intermediate wires (named anything you want) to connect the AND and OR
gates together. Note that the wire that feeds the NOT gate is really wire
`out`, so you do not necessarily need to declare a third wire here.
Notice how wires are driven by exactly one source (output of a gate), but
can feed multiple inputs.

The circuit is composed of two layers. The first layer, counting from the
input, is two AND gates: one whose input is connected to a and b, and the
second is connected to c and d. The second layer there is an OR gate to
OR the two AND outputs, connected the output 'out'. Additionally, there
is an inverted output 'out_n'.
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output out,
  output out_n
);

  wire w1, w2;
  assign w1 = a&b;
  assign w2 = c&d;
  assign out = w1|w2;
  assign out_n = ~out;

endmodule
```

#### model output
```verilog
module TopModule (
    input  a,
    input  b,
    input  c,
    input  d,
    output out,
    output out_n
);
    // Intermediate wires
    wire and_ab;
    wire and_cd;

    // First layer: AND gates
    and u_and1 (and_ab, a, b);
    and u_and2 (and_cd, c, d);

    // Second layer: OR gate
    or  u_or (out, and_ab, and_cd);

    // Inverted output
    not u_not (out_n, out);
endmodule
```
---

### Prob078_dualedge
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  d
 - output q

A dual-edge triggered flip-flop is triggered on both edges of the clock.
However, FPGAs don't have dual-edge triggered flip-flops, and using an
always @(posedge clk or negedge clk) is not accepted as a legal
sensitivity list. Build a circuit that functionally behaves like a
dual-edge triggered flip-flop.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input d,
  output reg q
);

  /*always @(posedge clk, negedge clk) begin
    q <= d;
  end*/

  reg qp, qn;
  always @(posedge clk)
    qp <= d;
  always @(negedge clk)
    qn <= d;

  // This causes q to change too early when clk changes. Need delay by
  // delta cycle
  // assign q = clk ? qp : qn;
  always @(*)
    q <= clk ? qp : qn;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire d,
    output wire q
);
    // Registers to capture data on each clock edge
    reg pos_reg;
    reg neg_reg;

    // Capture on rising edge
    always @(posedge clk) begin
        pos_reg <= d;
    end

    // Capture on falling edge
    always @(negedge clk) begin
        neg_reg <= d;
    end

    // Multiplex output based on clock level
    assign q = clk ? pos_reg : neg_reg;

endmodule
```
---

### Prob079_fsm3onehot
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in
 - input  state (4 bits)
 - output next_state (4 bits)
 - output out

The module should implement the state transition table for a Moore state
machine with one input, one output, and four states. Use the following
one-hot state encoding: A=4'b0001, B=4'b0010, C=4'b0100, D=4'b1000.
Derive state transition and output logic equations by inspection assuming
a one-hot encoding. Implement only the state transition logic and output
logic (the combinational logic portion) for this state machine.

  State | Next state in=0, Next state in=1 | Output
  A     | A, B                             | 0
  B     | C, B                             | 0
  C     | A, D                             | 0
  D     | C, B                             | 1
```
</details>

#### reference
```verilog
module RefModule (
  input in,
  input [3:0] state,
  output reg [3:0] next_state,
  output out
);

  parameter A=0, B=1, C=2, D=3;

  assign next_state[A] = (state[A] | state[C]) & ~in;
  assign next_state[B] = (state[A] | state[B] | state[D]) & in;
  assign next_state[C] = (state[B] | state[D]) & ~in;
  assign next_state[D] = state[C] & in;

  assign out = (state[D]);

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob080_timer
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk,
 - input  load,
 - input  data (10 bits)
 - output tc

The module should implement a timer that counts down for a given number
of clock cycles, then asserts a signal to indicate that the given
duration has elapsed. A good way to implement this is with a down-counter
that asserts an output signal when the count becomes 0. At each clock
cycle:

  (1) If load = 1, load the internal counter with the 10-bit data, the
  number of clock cycles the timer should count before timing out. The
  counter can be loaded at any time, including when it is still counting
  and has not yet reached 0.

  (2) If load = 0, the internal counter should decrement by 1. The output
  signal tc ("terminal count") indicates whether the internal counter has
  reached 0. Once the internal counter has reached 0, it should stay 0
  (stop counting) until the counter is loaded again.

The module should implement a single D flip-flop. Assume all sequential
logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input load,
  input [9:0] data,
  output tc
);

  logic [9:0] count_value;

  always @(posedge clk)
    if(load) count_value <= data;
    else if(count_value != 0) count_value <= count_value - 1;

  assign tc = count_value == 0;

endmodule
```

#### model output
```verilog
`timescale 1ns/1ps
`default_nettype none

module TopModule (
    input  wire        clk,
    input  wire        load,
    input  wire [9:0]  data,
    output wire        tc
);

    // 10‑bit down‑counter
    reg [9:0] count;

    // terminal count flag
    assign tc = (count == 10'd0);

    // Sequential logic: load or decrement
    always @(posedge clk) begin
        if (load) begin
            count <= data;                 // load new value
        end else if (count != 10'd0) begin
            count <= count - 10'd1;        // decrement when not zero
        end else begin
            count <= count;                // hold zero
        end
    end

endmodule
`default_nettype wire
```
---

### Prob081_7458
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  p1a
 - input  p1b
 - input  p1c
 - input  p1d
 - input  p1e
 - input  p1f
 - input  p2a
 - input  p2b
 - input  p2c
 - input  p2d
 - output p1y
 - output p2y

The 7458 is a chip with four AND gates and two OR gates. Implement a
module with the same functionality as the 7458 chip. It has 10 inputs and
2 outputs. You may choose to use an `assign` statement to drive each of
the output wires, or you may choose to declare (four) wires for use as
intermediate signals, where each internal wire is driven by the output of
one of the AND gates.

In this circuit, p1y should be the OR of two 3-input AND gates: one that
ANDs p1a, p1b, and p1c, and the second that ANDs p1d, p1e, and p1f. The
output p2y is the OR of two 2-input AND gates: one that ANDs p2a and p2b,
and the second that ANDs p2c and p2d.
```
</details>

#### reference
```verilog
module RefModule (
  input p1a,
  input p1b,
  input p1c,
  input p1d,
  input p1e,
  input p1f,
  output p1y,
  input p2a,
  input p2b,
  input p2c,
  input p2d,
  output p2y
);

  assign p1y = &{p1a, p1b, p1c} | &{p1d, p1e, p1f};
  assign p2y = &{p2a, p2b} | &{p2c, p2d};

endmodule
```

#### model output
```verilog
module TopModule(
    input  p1a,
    input  p1b,
    input  p1c,
    input  p1d,
    input  p1e,
    input  p1f,
    input  p2a,
    input  p2b,
    input  p2c,
    input  p2d,
    output p1y,
    output p2y
);
    // Internal wires for the AND gate outputs
    wire and1_out;
    wire and2_out;
    wire and3_out;
    wire and4_out;

    // 3‑input AND gates for p1y
    assign and1_out = p1a & p1b & p1c;
    assign and2_out = p1d & p1e & p1f;

    // 2‑input AND gates for p2y
    assign and3_out = p2a & p2b;
    assign and4_out = p2c & p2d;

    // OR the AND results to produce final outputs
    assign p1y = and1_out | and2_out;
    assign p2y = and3_out | and4_out;
endmodule
```
---

### Prob082_lfsr32
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output q (32 bits)

A linear feedback shift register is a shift register usually with a few
XOR gates to produce the next state of the shift register. A Galois LFSR
is one particular arrangement that shifts right, where a bit position with
a "tap" is XORed with the LSB output bit (q[0]) to produce its next value,
while bit positions without a tap shift right unchanged. 

The module should implement a 32-bit Galois LFSR with taps at bit
positions 32, 22, 2, and 1. Reset should be active high synchronous, and
should reset the output q to 32'h1. Assume all sequential logic is
triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output reg [31:0] q
);

  logic [31:0] q_next;
  always@(q) begin
    q_next = q[31:1];
    q_next[31] = q[0];
    q_next[21] ^= q[0];
    q_next[1] ^= q[0];
    q_next[0] ^= q[0];
  end

  always @(posedge clk) begin
    if (reset)
      q <= 32'h1;
    else
      q <= q_next;
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob083_mt2015_q4b
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  x
 - input  y
 - output z

The module can be described by the following simulation waveform:

  time  x  y  z
  0ns   0  0  1
  5ns   0  0  1
  10ns  0  0  1
  15ns  0  0  1
  20ns  0  0  1
  25ns  1  0  0
  30ns  1  0  0
  35ns  0  1  0
  40ns  0  1  0
  45ns  1  1  1
  50ns  1  1  1
  55ns  0  0  1
  60ns  0  1  0
  65ns  0  1  0
  70ns  1  1  1
  75ns  0  1  0
  80ns  0  1  0
  85ns  0  1  0
  90ns  1  0  0
```
</details>

#### reference
```verilog
module RefModule (
  input x,
  input y,
  output z
);

  assign z = ~(x^y);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire x,
    input  wire y,
    output wire z
);
    // z is XNOR of x and y
    assign z = ~(x ^ y);
endmodule
```
---

### Prob084_ece241_2013_q12
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  enable
 - input  S
 - input  A
 - input  B
 - input  C
 - output Z

The module should implement a circuit for an 8x1 memory, where writing to
the memory is accomplished by shifting-in bits, and reading is "random
access", as in a typical RAM. You will then use the circuit to realize a
3-input logic function. First, create an 8-bit shift register with 8
D-type flip-flops. Label the flip-flop outputs from Q[0]...Q[7]. The
shift register input should be called S, which feeds the input of Q[0]
(MSB is shifted in first). The enable input is synchronous active high
and controls whether to shift. Extend the circuit to have 3 additional
inputs A,B,C and an output Z. The circuit's behaviour should be as
follows: when ABC is 000, Z=Q[0], when ABC is 001, Z=Q[1], and so on.
Your circuit should contain ONLY the 8-bit shift register, and
multiplexers. Assume all sequential logic is triggered on the positive
edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input enable,
  input S,
  input A,
  input B,
  input C,
  output reg Z
);

  reg [7:0] q;
  always @(posedge clk) begin
    if (enable)
      q <= {q[6:0], S};
  end

  assign Z = q[ {A, B, C} ];

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire enable,
    input  wire S,
    input  wire A,
    input  wire B,
    input  wire C,
    output wire Z
);

    // 8‑bit shift register (Q[0] is the first/most‑significant bit shifted in)
    reg [7:0] Q;

    // Synchronous shift operation
    always @(posedge clk) begin
        if (enable) begin
            Q[0] <= S;
            Q[1] <= Q[0];
            Q[2] <= Q[1];
            Q[3] <= Q[2];
            Q[4] <= Q[3];
            Q[5] <= Q[4];
            Q[6] <= Q[5];
            Q[7] <= Q[6];
        end
        // else retain previous values (no else needed, registers hold their value)
    end

    // 8‑to‑1 multiplexer controlled by address ABC
    // ABC = 000 → Q[0], 001 → Q[1], … , 111 → Q[7]
    assign Z = Q[{A, B, C}];

endmodule
```
---

### Prob085_shift4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  load
 - input  ena
 - input  data (4 bits)
 - output q (4 bits)

The module should implement a 4-bit shift register (right shift), with
asynchronous positive edge triggered areset, synchronous active high
signals load, and enable.

  (1) areset: Resets shift register to zero.

  (2) load: Loads shift register with data[3:0] instead of shifting.

  (3) ena: Shift right (q[3] becomes zero, q[0] is shifted out and
       disappears).

  (4) q: The contents of the shift register. If both the load and ena
       inputs are asserted (1), the load input has higher priority.

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input load,
  input ena,
  input [3:0] data,
  output reg [3:0] q
);

  always @(posedge clk, posedge areset) begin
    if (areset)
      q <= 0;
    else if (load)
      q <= data;
    else if (ena)
      q <= q[3:1];
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        areset,
    input  wire        load,
    input  wire        ena,
    input  wire [3:0]  data,
    output reg  [3:0]  q
);

    // Asynchronous positive‑edge reset, synchronous load and enable
    always @(posedge clk or posedge areset) begin
        if (areset) begin
            q <= 4'b0;
        end else if (load) begin
            q <= data;
        end else if (ena) begin
            q <= {1'b0, q[3:1]};
        end else begin
            q <= q; // hold current state
        end
    end

endmodule
```
---

### Prob086_lfsr5
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output q (5 bits)

A linear feedback shift register is a shift register usually with a few
XOR gates to produce the next state of the shift register. A Galois LFSR
is one particular arrangement that shifts right, where a bit position with
a "tap" is XORed with the LSB output bit (q[0]) to produce its next value,
while bit positions without a tap shift right unchanged.  If the taps positions
are carefully chosen, the LFSR can be made to be "maximum-length". A maximum-length
LFSR of n bits cycles through 2**n-1 states before repeating (the all-zero state is
never reached).

The module should implement a 5-bit maximal-length Galois LFSR with taps
at bit positions 5 and 3. The active-high synchronous reset should reset
the LFSR output to 1. Assume all sequential logic is triggered on the
positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output reg [4:0] q
);

  logic [4:0] q_next;
  always @(q) begin
    q_next = q[4:1];
    q_next[4] = q[0];
    q_next[2] ^= q[0];
  end

  always @(posedge clk) begin
    if (reset)
      q <= 5'h1;
    else
      q <= q_next;
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob087_gates
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - output out_and
 - output out_or
 - output out_xor
 - output out_nand
 - output out_nor
 - output out_xnor
 - output out_anotb

The module should implement a combinational circuit with two inputs, a
and b. There are 7 outputs, each with a logic gate driving it:

  (1) out_and: a and b
  (2) out_or: a or b
  (3) out_xor: a xor b
  (4) out_nand: a nand b
  (5) out_nor: a nor b
  (6) out_xnor: a xnor b
  (7) out_anotb: a and-not b
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  output out_and,
  output out_or,
  output out_xor,
  output out_nand,
  output out_nor,
  output out_xnor,
  output out_anotb
);

  assign out_and = a&b;
  assign out_or = a|b;
  assign out_xor = a^b;
  assign out_nand = ~(a&b);
  assign out_nor = ~(a|b);
  assign out_xnor = a^~b;
  assign out_anotb = a & ~b;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    output wire out_and,
    output wire out_or,
    output wire out_xor,
    output wire out_nand,
    output wire out_nor,
    output wire out_xnor,
    output wire out_anotb
);
    assign out_and   = a & b;
    assign out_or    = a | b;
    assign out_xor   = a ^ b;
    assign out_nand  = ~(a & b);
    assign out_nor   = ~(a | b);
    assign out_xnor  = ~(a ^ b);
    assign out_anotb = a & ~b;
endmodule
```
---

### Prob088_ece241_2014_q5b
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  x
 - output z

The module should implement the following Mealy finite-state machine
which is an implementation of the 2's complementer. Implement using a
one-hot encoding. Resets into state A and reset is asynchronous
active-high.

  A --x=0 (z=0)--> A
  A --x=1 (z=1)--> B
  B --x=0 (z=1)--> B
  B --x=1 (z=0)--> B

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input x,
  output z
);

  parameter A=0,B=1;
  reg state;
  always @(posedge clk, posedge areset) begin
    if (areset)
      state <= A;
    else begin
      case (state)
        A: state <= x ? B : A;
        B: state <= B;
      endcase
    end
  end

  assign z = (state == A && x==1) | (state == B && x==0);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire areset,
    input  wire x,
    output reg  z
);

    // One-hot state encoding
    typedef enum logic [1:0] {
        STATE_A = 2'b01,
        STATE_B = 2'b10
    } state_t;

    state_t state, next_state;

    // Asynchronous active-high reset
    always @(posedge clk or posedge areset) begin
        if (areset)
            state <= STATE_A;
        else
            state <= next_state;
    end

    // Next-state logic
    always @* begin
        case (state)
            STATE_A: begin
                if (x)
                    next_state = STATE_B;
                else
                    next_state = STATE_A;
            end
            STATE_B: begin
                // Remain in B regardless of x
                next_state = STATE_B;
            end
            default: next_state = STATE_A;
        endcase
    end

    // Output logic (Mealy)
    always @* begin
        case (state)
            STATE_A: z = x;          // z = 0 when x=0, z = 1 when x=1
            STATE_B: z = ~x;         // z = 1 when x=0, z = 0 when x=1
            default: z = 1'b0;
        endcase
    end

endmodule
```
---

### Prob089_ece241_2014_q5a
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  x
 - output z

The module should implement a one-input one-output serial 2's
complementer Moore state machine. The input (x) is a series of bits (one
per clock cycle) beginning with the least-significant bit of the number,
and the output (Z) is the 2's complement of the input. The machine will
accept input numbers of arbitrary length. The circuit requires a positive
edge triggered asynchronous reset. The conversion begins when Reset is
released and stops when Reset is asserted. Assume all sequential logic is
triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input x,
  output z
);

  parameter A=0,B=1,C=2;
  reg [1:0] state;
  always @(posedge clk, posedge areset) begin
    if (areset)
      state <= A;
    else begin
      case (state)
        A: state <= x ? C : A;
        B: state <= x ? B : C;
        C: state <= x ? B : C;
      endcase
    end
  end

  assign z = (state == C);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire areset,
    input  wire x,
    output wire z
);
    // State: carry for the addition of 1 after bitwise inversion
    reg carry;
    reg z_reg;

    // Asynchronous positive-edge reset
    always @(posedge clk or posedge areset) begin
        if (areset) begin
            // Reset: initialize carry to 1 (for the +1 in two's complement)
            // Output is undefined during reset; set to 0 for safety
            carry <= 1'b1;
            z_reg <= 1'b0;
        end else begin
            // Moore output: register the computed bit
            z_reg <= (~x) ^ carry;
            // Update carry for next bit
            carry <= (~x) & carry;
        end
    end

    assign z = z_reg;
endmodule
```
---

### Prob090_circuit1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - output q

The module should implement a combinational circuit. Read the simulation
waveforms to determine what the circuit does, then implement it.

  time  a  b  q
  0ns   0  0  0
  5ns   0  0  0
  10ns  0  0  0
  15ns  0  0  0
  20ns  0  0  0
  25ns  0  1  0
  30ns  0  1  0
  35ns  1  0  0
  40ns  1  0  0
  45ns  1  1  1
  50ns  1  1  1
  55ns  0  0  0
  60ns  0  0  0
  65ns  0  1  0
  70ns  0  1  0
  75ns  1  0  0
  80ns  1  0  0
  85ns  1  1  1
  90ns  1  1  1
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  output q
);

  assign q = a&b;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    output wire q
);
    assign q = a & b;
endmodule
```
---

### Prob091_2012_q2b
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  y (6 bits)
 - input  w
 - output Y1
 - output Y3

Consider the following finite-state machine:

  A (0) --1--> B
  A (0) --0--> A
  B (0) --1--> C
  B (0) --0--> D
  C (0) --1--> E
  C (0) --0--> D
  D (0) --1--> F
  D (0) --0--> A
  E (1) --1--> E
  E (1) --0--> D
  F (1) --1--> C
  F (1) --0--> D

Assume that a one-hot code is used with the state assignment y[5:0] =
000001(A), 000010(B), 000100(C), 001000(D), 010000(E), 100000(F)

The module should implement the state output logic for this finite-state
machine. The output signal Y1 should be the input of state flip-flop
y[1]. The output signal Y3 should be the input of state flip-flop y[3].
Derive the implementation by inspection assuming the above one-hot
encoding.
```
</details>

#### reference
```verilog
module RefModule (
  input [5:0] y,
  input w,
  output Y1,
  output Y3
);

  assign Y1 = y[0]&w;
  assign Y3 = (y[1]|y[2]|y[4]|y[5]) & ~w;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [5:0] y,
    input  wire       w,
    output wire       Y1,
    output wire       Y3
);

    // Next-state logic for flip‑flop y[1] (state B)
    assign Y1 = y[0] & w;                     // A -> B on w=1

    // Next-state logic for flip‑flop y[3] (state D)
    assign Y3 = ~w & (y[1] | y[2] | y[4] | y[5]); // B,C,E,F -> D on w=0

endmodule
```
---

### Prob092_gatesv100
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in            (100 bits)
 - output out_both      (100 bits)
 - output out_any       (100 bits)
 - output out_different (100 bits)

The module takes as input a 100-bit input vector in[99:0] and should
produce the following three outputs:

  (1) out_both: Each bit of this output vector should indicate whether
  both the corresponding input bit and its neighbour to the left are '1'.
  For example, out_both[98] should indicate if in[98] and in[99] are both
  1. Since in[99] has no neighbour to the left, the answer is obvious so
  simply set out_both[99] to be zero.

  (2) out_any: Each bit of this output vector should indicate whether any
  of the corresponding input bit and its neighbour to the right are '1'.
  For example, out_any[2] should indicate if either in[2] or in[1] are 1.
  Since in[0] has no neighbour to the right, the answer is obvious so
  simply set out_any[0] to be zero.

  (3) out_different: Each bit of this output vector should indicate
  whether the corresponding input bit is different from its neighbour to
  the left. For example, out_different[98] should indicate if in[98] is
  different from in[99]. For this part, treat the vector as wrapping
  around, so in[99]'s neighbour to the left is in[0].
```
</details>

#### reference
```verilog
module RefModule (
  input  [99:0] in,
  output [99:0] out_both,
  output [99:0] out_any,
  output [99:0] out_different
);

  assign out_both = { 1'b0, (in[98:0] & in[99:1]) };

  assign out_any = { (in[98:0] | in[99:1]), 1'b0 };

  assign out_different = in ^ { in[0], in[99:1] };

endmodule
```

#### model output
```verilog

```
---

### Prob093_ece241_2014_q3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  c
 - input  d
 - output mux_in (4 bits)

For the following Karnaugh map, give the circuit implementation using one
4-to-1 multiplexer and as many 2-to-1 multiplexers as required, but using
as few as possible. You are not allowed to use any other logic gate and
you must use _a_ and _b_ as the multiplexer selector inputs, as shown on
the 4-to-1 multiplexer below.

      ab
  cd  00  01  11  10
  00 | 0 | 0 | 0 | 1 |
  01 | 1 | 0 | 0 | 0 |
  11 | 1 | 0 | 1 | 1 |
  10 | 1 | 0 | 0 | 1 |

Consider a block diagram with inputs 'c' and 'd' going into a module
called "TopModule". This "TopModule" has four outputs, mux_in[3:0], that
connect to a four input mux. The mux takes as input {a,b} and ab = 00 is
connected to mux_in[0], ab=01 is connected to mux_in[1], and so in. You
are implementing in Verilog just the portion labelled "TopModule", such
that the entire circuit (including the 4-to-1 mux) implements the K-map.
```
</details>

#### reference
```verilog
module RefModule (
  input c,
  input d,
  output [3:0] mux_in
);

  assign mux_in[0] = c | d;
  assign mux_in[1] = 0;
  assign mux_in[2] = ~d;
  assign mux_in[3] = c&d;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob094_gatesv
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in            (4 bits)
 - output out_both      (4 bits)
 - output out_any       (4 bits)
 - output out_different (4 bits)

You are given a four-bit input vector. We want to know some relationships
between each bit and its neighbour:

  (1) out_both: Each bit of this output vector should indicate whether
  both the corresponding input bit and its neighbour to the left (higher
  index) are '1'. For example, out_both[2] should indicate if in[2] and
  in[3] are both 1. Since in[3] has no neighbour to the left, the answer
  is obvious so we don't need to know out_both[3].

  (2) out_any: Each bit of this output vector should indicate whether any
  of the corresponding input bit and its neighbour to the right are '1'.
  For example, out_any[2] should indicate if either in[2] or in[1] are 1.
  Since in[0] has no neighbour to the right, the answer is obvious so we
  don't need to know out_any[0].

  (3) out_different: Each bit of this output vector should indicate
  whether the corresponding input bit is different from its neighbour to
  the left. For example, out_different[2] should indicate if in[2] is
  different from in[3]. For this part, treat the vector as wrapping
  around, so in[3]'s neighbour to the left is in[0].
```
</details>

#### reference
```verilog
module RefModule (
  input  [3:0] in,
  output [3:0] out_both,
  output [3:0] out_any,
  output [3:0] out_different
);

  assign out_both[2:0] = in[2:0] & in[3:1];
  assign out_any[3:1]  = in[2:0] | in[3:1];
  assign out_different = in^{in[0], in[3:1]};

  // we don't care about out_both[3] or out_any[0]
  assign out_both[3] = 1'bx;
  assign out_any[0]  = 1'bx;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [3:0] in,
    output wire [3:0] out_both,
    output wire [3:0] out_any,
    output wire [3:0] out_different
);

    // (1) Both current bit and neighbour to the left (higher index)
    assign out_both[0] = in[0] & in[1];
    assign out_both[1] = in[1] & in[2];
    assign out_both[2] = in[2] & in[3];
    assign out_both[3] = 1'b0; // not needed

    // (2) Any of current bit or neighbour to the right (lower index)
    assign out_any[0] = 1'b0; // not needed
    assign out_any[1] = in[1] | in[0];
    assign out_any[2] = in[2] | in[1];
    assign out_any[3] = in[3] | in[2];

    // (3) Different from neighbour to the left, with wrap‑around
    assign out_different[0] = in[0] ^ in[1];
    assign out_different[1] = in[1] ^ in[2];
    assign out_different[2] = in[2] ^ in[3];
    assign out_different[3] = in[3] ^ in[0];

endmodule
```
---

### Prob095_review2015_fsmshift
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - output shift_ena

This module is a part of the FSM for controlling a shift register, we
want the ability to enable the shift register for exactly 4 clock cycles
whenever the proper bit pattern is detected. Whenever the FSM is reset,
assert shift_ena for 4 cycles, then 0 forever (until reset). Reset should
be active high synchronous.

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  output shift_ena
);

  parameter B0=0, B1=1, B2=2, B3=3, Done=4;

  reg [2:0] state, next;

  always_comb begin
    case (state)
      B0: next = B1;
      B1: next = B2;
      B2: next = B3;
      B3: next = Done;
      Done: next = Done;
    endcase
  end

  always @(posedge clk)
    if (reset) state <= B0;
    else state <= next;

  assign shift_ena = (state == B0 || state == B1 || state == B2 || state == B3);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,      // active high synchronous reset
    output wire shift_ena
);

    // Counter to generate a 4‑cycle enable pulse after reset
    reg [1:0] cnt;
    reg       shift_ena_reg;

    assign shift_ena = shift_ena_reg;

    always @(posedge clk) begin
        if (reset) begin
            cnt           <= 2'd0;
            shift_ena_reg <= 1'b1;   // start 4‑cycle enable
        end else begin
            if (shift_ena_reg) begin
                if (cnt < 2'd3) begin
                    cnt           <= cnt + 2'd1;
                    shift_ena_reg <= 1'b1; // keep enabled
                end else begin
                    cnt           <= cnt;   // hold count
                    shift_ena_reg <= 1'b0; // disable after 4 cycles
                end
            end else begin
                cnt <= cnt; // remain idle
            end
        end
    end

endmodule
```
---

### Prob096_review2015_fsmseq
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  data
 - output start_shifting

The module should implement a finite-state machine that searches for the
sequence 1101 in an input bit stream. When the sequence is found, it
should set start_shifting to 1, forever, until reset. Reset is active
high synchronous. Assume all sequential logic is triggered on the
positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input data,
  output start_shifting
);

  parameter S=0, S1=1, S11=2, S110=3, Done=4;

  reg [2:0] state, next;

  always_comb begin
    case (state)
      S: next = data ? S1: S;
      S1: next = data ? S11: S;
      S11: next = data ? S11 : S110;
      S110: next = data ? Done : S;
      Done: next = Done;
    endcase
  end

  always @(posedge clk)
    if (reset) state <= S;
    else state <= next;

  assign start_shifting = state == Done;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire data,
    output reg  start_shifting
);

    // State encoding
    typedef enum logic [2:0] {
        IDLE       = 3'd0,
        S1         = 3'd1, // saw '1'
        S11        = 3'd2, // saw '11'
        S110       = 3'd3, // saw '110'
        FOUND      = 3'd4  // sequence detected
    } state_t;

    state_t state, next_state;

    // Sequential logic: state register and output latch
    always @(posedge clk) begin
        if (reset) begin
            state          <= IDLE;
            start_shifting <= 1'b0;
        end else begin
            state <= next_state;
            if (state == FOUND)
                start_shifting <= 1'b1;
        end
    end

    // Combinational logic: next state logic
    always @* begin
        next_state = state; // default stay in current state
        case (state)
            IDLE: begin
                if (data) next_state = S1;
                else     next_state = IDLE;
            end
            S1: begin
                if (data) next_state = S11;
                else     next_state = IDLE;
            end
            S11: begin
                if (data) next_state = S11; // stay in S11 on consecutive '1's
                else     next_state = S110;
            end
            S110: begin
                if (data) next_state = FOUND;
                else     next_state = IDLE;
            end
            FOUND: begin
                // Remain in FOUND forever (until reset)
                next_state = FOUND;
            end
            default: next_state = IDLE;
        endcase
    end

endmodule
```
---

### Prob097_mux9to1v
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a   (16 bits)
 - input  b   (16 bits)
 - input  c   (16 bits)
 - input  d   (16 bits)
 - input  e   (16 bits)
 - input  f   (16 bits)
 - input  g   (16 bits)
 - input  h   (16 bits)
 - input  i   (16 bits)
 - input  sel ( 4 bits)
 - output out (16 bits)

The module should implement a 16-bit wide, 9-to-1 multiplexer. sel=0
chooses a, sel=1 chooses b, etc. For the unused cases (sel=9 to 15), set
all output bits to '1'.
```
</details>

#### reference
```verilog
module RefModule (
  input [15:0] a,
  input [15:0] b,
  input [15:0] c,
  input [15:0] d,
  input [15:0] e,
  input [15:0] f,
  input [15:0] g,
  input [15:0] h,
  input [15:0] i,
  input [3:0] sel,
  output logic [15:0] out
);

  always @(*) begin
    out = '1;
    case (sel)
      4'h0: out = a;
      4'h1: out = b;
      4'h2: out = c;
      4'h3: out = d;
      4'h4: out = e;
      4'h5: out = f;
      4'h6: out = g;
      4'h7: out = h;
      4'h8: out = i;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  [15:0] a,
    input  [15:0] b,
    input  [15:0] c,
    input  [15:0] d,
    input  [15:0] e,
    input  [15:0] f,
    input  [15:0] g,
    input  [15:0] h,
    input  [15:0] i,
    input  [3:0]  sel,
    output reg [15:0] out
);
    always @* begin
        case (sel)
            4'd0: out = a;
            4'd1: out = b;
            4'd2: out = c;
            4'd3: out = d;
            4'd4: out = e;
            4'd5: out = f;
            4'd6: out = g;
            4'd7: out = h;
            4'd8: out = i;
            default: out = 16'hFFFF;
        endcase
    end
endmodule
```
---

### Prob098_circuit7
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  a
 - output q

This is a sequential circuit. Read the simulation waveforms to determine
what the circuit does, then implement it.

  time  clk a   q
  0ns   0   x   x
  5ns   1   0   x
  10ns  0   0   x
  15ns  1   0   1
  20ns  0   0   1
  25ns  1   0   1
  30ns  0   0   1
  35ns  1   1   1
  40ns  0   1   1
  45ns  1   1   0
  50ns  0   1   0
  55ns  1   1   0
  60ns  0   1   0
  65ns  1   1   0
  70ns  0   1   0
  75ns  1   1   0
  80ns  0   1   0
  85ns  1   1   0
  90ns  0   1   0

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input a,
  output reg q
);

  always @(posedge clk)
    q <= ~a;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob099_m2014_q6c
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  y (6 bits)
 - input  w
 - output Y1
 - output Y3

Consider the state machine shown below:

  A (0) --0--> B
  A (0) --1--> A
  B (0) --0--> C
  B (0) --1--> D
  C (0) --0--> E
  C (0) --1--> D
  D (0) --0--> F
  D (0) --1--> A
  E (1) --0--> E
  E (1) --1--> D
  F (1) --0--> C
  F (1) --1--> D

Resets into state A. For this part, assume that a one-hot code is used
with the state assignment y[5:0] = 000001, 000010, 000100, 001000,
010000, 100000 for states A, B,..., F, respectively.

The module shou module ment the next-state signals Y2 and Y4
corresponding to signal y[1] and y[3]. Derive the logic equations by
inspection assuming the one-hot encoding.
 implement the next-state signals  and corresponding to
signal y[1] and y[3]Derive the logic equations byinspection assuming the one-hot encoding.
```
</details>

#### reference
```verilog
module RefModule (
  input  [5:0] y,
  input        w,
  output       Y1,
  output       Y3
);

  assign Y1 = y[0]&~w;
  assign Y3 = (y[1]|y[2]|y[4]|y[5]) & w;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob100_fsm3comb
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in
 - input  state (2 bits)
 - output next_state (2 bits)
 - output out

The module should implement the following is the state transition table
for a Moore state machine with one input, one output, and four states.
Use the following state encoding: A=2'b00, B=2'b01, C=2'b10,
D=2'b11.Implement only the state transition logic and output logic (the
combinational logic portion) for this state machine. Given the current
state (state), compute the next_state and output (out) based on the state
transition table.

  State | Next state in=0, Next state in=1 | Output
  A     | A, B                             | 0
  B     | C, B                             | 0
  C     | A, D                             | 0
  D     | C, B                             | 1
```
</details>

#### reference
```verilog
module RefModule (
  input in,
  input [1:0] state,
  output reg [1:0] next_state,
  output out
);

  parameter A=0, B=1, C=2, D=3;

  always_comb begin
    case (state)
      A: next_state = in ? B : A;
      B: next_state = in ? B : C;
      C: next_state = in ? D : A;
      D: next_state = in ? B : C;
    endcase
  end

  assign out = (state==D);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire       in,
    input  wire [1:0] state,
    output reg  [1:0] next_state,
    output reg        out
);
    // Combinational state‑transition and output logic (Moore machine)
    always @* begin
        // Default values
        out        = 1'b0;
        next_state = 2'b00;

        case (state)
            2'b00: begin // A
                out = 1'b0;
                if (in)
                    next_state = 2'b01; // B
                else
                    next_state = 2'b00; // A
            end

            2'b01: begin // B
                out = 1'b0;
                if (in)
                    next_state = 2'b01; // B
                else
                    next_state = 2'b10; // C
            end

            2'b10: begin // C
                out = 1'b0;
                if (in)
                    next_state = 2'b11; // D
                else
                    next_state = 2'b00; // A
            end

            2'b11: begin // D
                out = 1'b1;
                if (in)
                    next_state = 2'b01; // B
                else
                    next_state = 2'b10; // C
            end

            default: begin
                out        = 1'b0;
                next_state = 2'b00;
            end
        endcase
    end
endmodule
```
---

### Prob101_circuit4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - output q

The module should implement a combinational circuit. Read the simulation
waveforms to determine what the circuit does, then implement it.

  time  a  b  c  d  q
  0ns   0  0  0  0  0
  5ns   0  0  0  0  0
  10ns  0  0  0  0  0
  15ns  0  0  0  0  0
  20ns  0  0  0  1  0
  25ns  0  0  1  0  1
  30ns  0  0  1  1  1
  35ns  0  1  0  0  1
  40ns  0  1  0  1  1
  45ns  0  1  1  0  1
  50ns  0  1  1  1  1
  55ns  1  0  0  0  0
  60ns  1  0  0  1  0
  65ns  1  0  1  0  1
  70ns  1  0  1  1  1
  75ns  1  1  0  0  1
  80ns  1  1  0  1  1
  85ns  1  1  1  0  1
  90ns  1  1  1  1  1
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output q
);

  assign q = c | b;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    input  wire c,
    input  wire d,
    output wire q
);
    // Combinational logic: q = b OR c
    assign q = b | c;
endmodule
```
---

### Prob102_circuit3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - output q

The module should implement a combinational circuit. Read the simulation
waveforms to determine what the circuit does, then implement it.

  time  a  b  c  d  q
  0ns   0  0  0  0  0
  5ns   0  0  0  0  0
  10ns  0  0  0  0  0
  15ns  0  0  0  0  0
  20ns  0  0  0  1  0
  25ns  0  0  1  0  0
  30ns  0  0  1  1  0
  35ns  0  1  0  0  0
  40ns  0  1  0  1  1
  45ns  0  1  1  0  1
  50ns  0  1  1  1  1
  55ns  1  0  0  0  0
  60ns  1  0  0  1  1
  65ns  1  0  1  0  1
  70ns  1  0  1  1  1
  75ns  1  1  0  0  0
  80ns  1  1  0  1  1
  85ns  1  1  1  0  1
  90ns  1  1  1  1  1
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output q
);

  assign q = (a|b) & (c|d);

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob103_circuit2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - output q

The module should implement a combinational circuit. Read the simulation
waveforms to determine what the circuit does, then implement it.

  time  a  b  c  d  q
  0ns   0  0  0  0  1
  5ns   0  0  0  0  1
  10ns  0  0  0  0  1
  15ns  0  0  0  0  1
  20ns  0  0  0  1  0
  25ns  0  0  1  0  0
  30ns  0  0  1  1  1
  35ns  0  1  0  0  0
  40ns  0  1  0  1  1
  45ns  0  1  1  0  1
  50ns  0  1  1  1  0
  55ns  1  0  0  0  0
  60ns  1  0  0  1  1
  65ns  1  0  1  0  1
  70ns  1  0  1  1  0
  75ns  1  1  0  0  1
  80ns  1  1  0  1  0
  85ns  1  1  1  0  0
  90ns  1  1  1  1  1
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output q
);

  assign q = ~a^b^c^d;

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    input  wire c,
    input  wire d,
    output wire q
);
    // q is 1 when the number of high inputs is even (even parity)
    assign q = ~(a ^ b ^ c ^ d);
endmodule
```
---

### Prob104_mt2015_muxdff
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  L
 - input  q_in
 - input  r_in
 - output Q

Consider this Verilog module "full_module":

  module full_module (
      input [2:0] r,
      input L,
      input clk,
      output reg [2:0] q);

    always @(posedge clk) begin
      if (L) begin
        q <= r;
      end else begin
        q <= {q[1] ^ q[2], q[0], q[2]};
      end
    end

  endmodule

Note that q[2:0] is three bits wide, representing three flip-flops that can be
loaded from r when L is asserted. You want to factor full_module into a hierarchical
design, flipflop and 2:1 multiplexer are in a submodule "TopModule", and that submodule
will be instantiated three times in full_module code. Create the submodule called "TopModule".
You do not have to provide the revised full_module.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input L,
  input q_in,
  input r_in,
  output reg Q
);

  initial Q=0;
  always @(posedge clk)
    Q <= L ? r_in : q_in;

endmodule
```

#### model output
```verilog
module TopModule (
    input  clk,
    input  L,
    input  q_in,
    input  r_in,
    output reg Q
);
    always @(posedge clk) begin
        if (L)
            Q <= r_in;
        else
            Q <= q_in;
    end
endmodule
```
---

### Prob105_rotate100
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  load
 - input  ena  (  2 bits)
 - input  data (100 bits)
 - output q    (100 bits)

The module should implement a 100-bit left/right rotator, with
synchronous load and left/right enable. A rotator shifts-in the
shifted-out bit from the other end of the register, unlike a shifter that
discards the shifted-out bit and shifts in a zero. If enabled, a rotator
rotates the bits around and does not modify/discard them.

  (1) load: Loads shift register with data[99:0] instead of rotating.
      Synchronous active high.

  (2) ena[1:0]: Synchronous. Chooses whether and which direction to
      rotate:
      (a) 2'b01 rotates right by one bit,
      (b) 2'b10 rotates left by one bit,
      (c) 2'b00 and 2'b11 do not rotate.

  (3) q: The contents of the rotator.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input load,
  input [1:0] ena,
  input [99:0] data,
  output reg [99:0] q
);

  always @(posedge clk) begin
    if (load)
      q <= data;
    else if (ena == 2'h1)
      q <= {q[0], q[99:1]};
    else if (ena == 2'h2)
      q <= {q[98:0], q[99]};
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        load,
    input  wire [1:0]  ena,
    input  wire [99:0] data,
    output reg  [99:0] q
);

always @(posedge clk) begin
    if (load) begin
        q <= data;
    end else begin
        case (ena)
            2'b01: q <= {q[0], q[99:1]};   // rotate right by 1
            2'b10: q <= {q[98:0], q[99]};  // rotate left  by 1
            default: q <= q;               // hold (no rotation)
        endcase
    end
end

endmodule
```
---

### Prob106_always_nolatches
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  scancode (16 bits)
 - output left
 - output down
 - output right
 - output up

The module should process scancodes from a PS/2 keyboard for a game.
Given the last two bytes of scancodes received, you need to indicate
whether one of the arrow keys on the keyboard have been pressed. This
involves a fairly simple mapping, which can be implemented as a case
statement (or if-elseif) with four cases.

  Scancode[15:0] | Arrow key
  16'he06b       | left arrow
  16'he072       | down arrow
  16'he074       | right arrow
  16'he075       | up arrow
  Anything else  | none

Your circuit has one 16-bit input, and four outputs. Build this circuit
that recognizes these four scancodes and asserts the correct output.
```
</details>

#### reference
```verilog
module RefModule (
  input [15:0] scancode,
  output reg left,
  output reg down,
  output reg right,
  output reg up
);

  always @(*) begin
    {up, left, down, right} = 0;
    case (scancode)
      16'he06b: left = 1;
      16'he072: down = 1;
      16'he074: right = 1;
      16'he075: up = 1;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  [15:0] scancode,
    output reg    left,
    output reg    down,
    output reg    right,
    output reg    up
);
    // Combinational logic to decode arrow key scancodes
    always @* begin
        // Default: no arrow pressed
        left  = 1'b0;
        down  = 1'b0;
        right = 1'b0;
        up    = 1'b0;

        case (scancode)
            16'he06b: left  = 1'b1; // left arrow
            16'he072: down  = 1'b1; // down arrow
            16'he074: right = 1'b1; // right arrow
            16'he075: up    = 1'b1; // up arrow
            default: ;               // keep defaults
        endcase
    end
endmodule
```
---

### Prob107_fsm1s
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  in
 - output out

Implement the following Moore state machine with two states, one input,
and one output. The reset state is B and reset is active-high
synchronous.

  B (out=1) --in=0--> A
  B (out=1) --in=1--> B
  A (out=0) --in=0--> B
  A (out=0) --in=1--> A
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input in,
  input reset,
  output out
);

  parameter A=0, B=1;
  reg state;
  reg next;

    always_comb begin
    case (state)
      A: next = in ? A : B;
      B: next = in ? B : A;
    endcase
    end

    always @(posedge clk) begin
    if (reset) state <= B;
        else state <= next;
  end

  assign out = (state==B);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire in,
    output wire out
);
    // State encoding
    localparam STATE_B = 1'b0;
    localparam STATE_A = 1'b1;

    reg state;

    // State register with synchronous active‑high reset
    always @(posedge clk) begin
        if (reset) begin
            state <= STATE_B;
        end else begin
            case (state)
                STATE_B: state <= (in) ? STATE_B : STATE_A;
                STATE_A: state <= (in) ? STATE_A : STATE_B;
                default: state <= STATE_B;
            endcase
        end
    end

    // Moore output logic
    assign out = (state == STATE_B) ? 1'b1 : 1'b0;

endmodule
```
---

### Prob108_rule90
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk,
 - input  load,
 - input  data (512 bits)
 - output q    (512 bits)

The module should implement Rule 90, a one-dimensional cellular automaton
with interesting properties. The rules are simple. There is a
one-dimensional array of cells (on or off). At each time step, the next
state of each cell is the XOR of the cell's two current neighbours:

  Left | Center | Right | Center's next state
  1    | 1      | 1     | 0
  1    | 1      | 0     | 1
  1    | 0      | 1     | 0
  1    | 0      | 0     | 1
  0    | 1      | 1     | 1
  0    | 1      | 0     | 0
  0    | 0      | 1     | 1
  0    | 0      | 0     | 0

In this circuit, create a 512-cell system (q[511:0]), and advance by one
time step each clock cycle. The load input indicates the state of the
system should be loaded with data[511:0]. Assume the boundaries (q[-1]
and q[512]) are both zero (off). Assume all sequential logic is triggered
on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input load,
  input [511:0] data,
  output reg [511:0] q
);

  always @(posedge clk) begin
    if (load)
      q <= data;
    else begin
      q <= q[$bits(q)-1:1] ^ {q[$bits(q)-2:0], 1'b0} ;
    end
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire        clk,
    input  wire        load,
    input  wire [511:0] data,
    output reg  [511:0] q
);

    // combinational next‑state logic for Rule 90
    wire [511:0] next_state;

    assign next_state[0]   = q[1];        // left boundary is 0
    assign next_state[511] = q[510];      // right boundary is 0

    genvar i;
    generate
        for (i = 1; i < 511; i = i + 1) begin : gen_rule90
            assign next_state[i] = q[i-1] ^ q[i+1];
        end
    endgenerate

    // sequential update
    always @(posedge clk) begin
        if (load)
            q <= data;
        else
            q <= next_state;
    end

endmodule
```
---

### Prob109_fsm1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  in
 - output out

The module should implement a Moore machine with the diagram described
below:

  B (1) --0--> A
  B (1) --1--> B
  A (0) --0--> B
  A (0) --1--> A

It should asynchronously reset into state B if reset if high.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input in,
  input areset,
  output out
);

  parameter A=0, B=1;
  reg state;
  reg next;

    always_comb begin
    case (state)
      A: next = in ? A : B;
      B: next = in ? B : A;
    endcase
    end

    always @(posedge clk, posedge areset) begin
    if (areset) state <= B;
        else state <= next;
  end

  assign out = (state==B);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire areset,
    input  wire in,
    output wire out
);

    // State encoding
    localparam STATE_A = 1'b0;
    localparam STATE_B = 1'b1;

    // State register
    reg state_reg, state_next;

    // Next‑state logic (combinational)
    always @* begin
        case (state_reg)
            STATE_B: begin
                if (in == 1'b0)
                    state_next = STATE_A;
                else
                    state_next = STATE_B;
            end
            STATE_A: begin
                if (in == 1'b0)
                    state_next = STATE_B;
                else
                    state_next = STATE_A;
            end
            default: state_next = STATE_B; // safety fallback
        endcase
    end

    // State register update (sequential) with asynchronous reset
    always @(posedge clk or posedge areset) begin
        if (areset)
            state_reg <= STATE_B;   // reset to state B
        else
            state_reg <= state_next;
    end

    // Moore output: output reflects current state
    assign out = state_reg;

endmodule
```
---

### Prob110_fsm2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  j
 - input  k
 - output out

The module should implement a Moore state machine with two states, two
inputs, and one output according to diagram described below. Reset is an
active-high asynchronous reset to state OFF.

  OFF (out=0) --j=0--> OFF
  OFF (out=0) --j=1--> ON
  ON  (out=1) --k=0--> ON
  ON  (out=1) --k=1--> OFF
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input j,
  input k,
  input areset,
  output out
);

  parameter A=0, B=1;
  reg state;
  reg next;

    always_comb begin
    case (state)
      A: next = j ? B : A;
      B: next = k ? A : B;
    endcase
    end

    always @(posedge clk, posedge areset) begin
    if (areset) state <= A;
        else state <= next;
  end

  assign out = (state==B);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire areset,
    input  wire j,
    input  wire k,
    output wire out
);

    // State encoding
    localparam OFF = 1'b0;
    localparam ON  = 1'b1;

    // State register
    reg state;

    // Next-state logic
    always @(posedge clk or posedge areset) begin
        if (areset) begin
            state <= OFF;
        end else begin
            case (state)
                OFF: state <= j ? ON : OFF;
                ON : state <= k ? OFF : ON;
                default: state <= OFF;
            endcase
        end
    end

    // Moore output
    assign out = state;

endmodule
```
---

### Prob111_fsm2s
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  j
 - input  k
 - output out

The module should implement a Moore state machine with two states, two
inputs, and one output according to diagram described below. Reset is an
active-high synchronous reset to state OFF.

  OFF (out=0) --j=0--> OFF
  OFF (out=0) --j=1--> ON
  ON  (out=1) --k=0--> ON
  ON  (out=1) --k=1--> OFF
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input j,
  input k,
  input reset,
  output out
);

  parameter A=0, B=1;
  reg state;
  reg next;

    always_comb begin
    case (state)
      A: next = j ? B : A;
      B: next = k ? A : B;
    endcase
    end

    always @(posedge clk) begin
    if (reset) state <= A;
        else state <= next;
  end

  assign out = (state==B);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire j,
    input  wire k,
    output wire out
);
    // State encoding
    localparam OFF = 1'b0;
    localparam ON  = 1'b1;

    reg state, next_state;

    // State register with synchronous active‑high reset
    always @(posedge clk) begin
        if (reset)
            state <= OFF;
        else
            state <= next_state;
    end

    // Next‑state combinational logic
    always @* begin
        case (state)
            OFF: next_state = j ? ON : OFF;
            ON : next_state = k ? OFF : ON;
            default: next_state = OFF;
        endcase
    end

    // Moore output
    assign out = state;

endmodule
```
---

### Prob112_always_case2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in  (4 bits)
 - output pos (2 bits)

The module should implement a priority encoder. A priority encoder is a
combinational circuit that, when given an input bit vector, outputs the
position of the first 1 bit in the vector. For example, a 8-bit priority
encoder given the input 8'b10010000 would output 3'd4, because bit[4] is
first bit that is high. Build a 4-bit priority encoder. For this problem,
if none of the input bits are high (i.e., input is zero), output zero.
Note that a 4-bit number has 16 possible combinations.
```
</details>

#### reference
```verilog
module RefModule (
  input [3:0] in,
  output reg [1:0] pos
);

  always @(*) begin
    case (in)
      4'h0: pos = 2'h0;
      4'h1: pos = 2'h0;
      4'h2: pos = 2'h1;
      4'h3: pos = 2'h0;
      4'h4: pos = 2'h2;
      4'h5: pos = 2'h0;
      4'h6: pos = 2'h1;
      4'h7: pos = 2'h0;
      4'h8: pos = 2'h3;
      4'h9: pos = 2'h0;
      4'ha: pos = 2'h1;
      4'hb: pos = 2'h0;
      4'hc: pos = 2'h2;
      4'hd: pos = 2'h0;
      4'he: pos = 2'h1;
      4'hf: pos = 2'h0;
      default: pos = 2'b0;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  [3:0] in,
    output reg [1:0] pos
);
    always @* begin
        if (in[0])       pos = 2'd0;
        else if (in[1])  pos = 2'd1;
        else if (in[2])  pos = 2'd2;
        else if (in[3])  pos = 2'd3;
        else             pos = 2'd0;
    end
endmodule
```
---

### Prob113_2012_q1g
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  x (4 bits)
 - output f

The module should implement the function f shown in the Karnaugh map
below.

             x[0]x[1]
x[2]x[3]  00  01  11  10
  00     | 1 | 0 | 0 | 1 |
  01     | 0 | 0 | 0 | 0 |
  11     | 1 | 1 | 1 | 0 |
  10     | 1 | 1 | 0 | 1 |
```
</details>

#### reference
```verilog
module RefModule (
  input [3:0] x,
  output logic f
);

  always_comb begin
    case (x)
      4'h0: f = 1;
      4'h1: f = 1;
      4'h2: f = 0;
      4'h3: f = 0;
      4'h4: f = 1;
      4'h5: f = 1;
      4'h6: f = 1;
      4'h7: f = 0;
      4'h8: f = 0;
      4'h9: f = 0;
      4'ha: f = 0;
      4'hb: f = 0;
      4'hc: f = 1;
      4'hd: f = 0;
      4'he: f = 1;
      4'hf: f = 1;
    endcase
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob114_bugs_case
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  code (8 bits)
 - output out  (4 bits)
 - output valid

The module should recognize 8-bit keyboard scancodes for keys 0 through
9. It should indicate whether one of the 10 cases were recognized
(valid), and if so, which key was detected. If the 8-bit input is 8'h45,
8'h16, 8'h1e, 8'h26, 8'h25, 8'h2e, 8'h36, 8'h3d, 8'h3e, or 8'h46, the
4-bit output will be set to 0, 1, 2, 3, 4, 5, 6, 7, 8, or 9 respectively,
the 1-bit valid would be set to 1. If the input does not match any of the
cases, both output signals would be set to 0.
```
</details>

#### reference
```verilog
module RefModule (
  input [7:0] code,
  output reg [3:0] out,
  output reg valid
);

  // uhh.. make a case statement: maps scancode to 0-9, but accidentally
  // infer a latch? and have one of the entries be wrong? (duplicate
  // case, using different base!)

  always @(*) begin
    out = 0;
    valid = 1;
    case (code)
      8'h45: out = 0;
      8'h16: out = 1;
      8'h1e: out = 2;
      8'h26: out = 3;
      8'h25: out = 4;
      8'h2e: out = 5;
      8'h36: out = 6;
      8'h3d: out = 7;
      8'h3e: out = 8;
      8'h46: out = 9;
      default: valid = 0;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [7:0] code,
    output reg  [3:0] out,
    output reg        valid
);

    // Combinational decoder for keyboard scancodes 0‑9
    always @(*) begin
        case (code)
            8'h45: begin out = 4'd0; valid = 1'b1; end // '0'
            8'h16: begin out = 4'd1; valid = 1'b1; end // '1'
            8'h1e: begin out = 4'd2; valid = 1'b1; end // '2'
            8'h26: begin out = 4'd3; valid = 1'b1; end // '3'
            8'h25: begin out = 4'd4; valid = 1'b1; end // '4'
            8'h2e: begin out = 4'd5; valid = 1'b1; end // '5'
            8'h36: begin out = 4'd6; valid = 1'b1; end // '6'
            8'h3d: begin out = 4'd7; valid = 1'b1; end // '7'
            8'h3e: begin out = 4'd8; valid = 1'b1; end // '8'
            8'h46: begin out = 4'd9; valid = 1'b1; end // '9'
            default: begin out = 4'd0; valid = 1'b0; end
        endcase
    end

endmodule
```
---

### Prob115_shift18
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  load
 - input  ena
 - input  amount (2 bits)
 - input  data (64 bits)
 - output q (64 bits)

The module should implement a 64-bit arithmetic shift register, with
synchronous load. The shifter can shift both left and right, and by 1 or
8 bit positions, selected by "amount." Assume the right shit is an
arithmetic right shift.

Signals are defined as below:

  (1) load: Loads shift register with data[63:0] instead of shifting.
       Active high.
  (2) ena: Chooses whether to shift. Active high.
  (3) amount: Chooses which direction and how much to shift.
      (a) 2'b00: shift left by 1 bit.
      (b) 2'b01: shift left by 8 bits.
      (c) 2'b10: shift right by 1 bit.
      (d) 2'b11: shift right by 8 bits.
  (4) q: The contents of the shifter.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input load,
  input ena,
  input [1:0] amount,
  input [63:0] data,
  output reg [63:0] q
);

  always @(posedge clk) begin
    if (load)
      q <= data;
    else if (ena) case (amount)
      2'b00: q <= {q[62:0], 1'b0};
      2'b01: q <= {q[55:0], 8'b0};
      2'b10: q <= {q[63], q[63:1]};
      2'b11: q <= {{8{q[63]}}, q[63:8]};
      default: q <= 64'hx;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input               clk,
    input               load,
    input               ena,
    input        [1:0]  amount,
    input       [63:0]  data,
    output reg  [63:0]  q
);

always @(posedge clk) begin
    if (load) begin
        q <= data;
    end else if (ena) begin
        case (amount)
            2'b00: q <= {q[62:0], 1'b0};                     // shift left 1
            2'b01: q <= {q[55:0], 8'b0};                     // shift left 8
            2'b10: q <= {q[63], q[63:1]};                    // arithmetic shift right 1
            2'b11: q <= {{8{q[63]}}, q[63:8]};               // arithmetic shift right 8
            default: q <= q;
        endcase
    end
    // else retain current value (no else needed, q holds its value)
end

endmodule
```
---

### Prob116_m2014_q3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  x (4 bits)
 - output f

The module should implement the function f shown in the Karnaugh map
below. d is don't-care, which means you may choose to output whatever
value is convenient.

                x[1]x[2]
  x[3]x[4]   00  01  11  10
        00 | d | 0 | d | d |
        01 | 0 | d | 1 | 0 |
        11 | 1 | 1 | d | d |
        10 | 1 | 1 | 0 | d |
```
</details>

#### reference
```verilog
module RefModule (
  input [3:0] x,
  output logic f
);

  always_comb begin
    case (x)
      4'h0: f = 1'bx;
      4'h1: f = 1'bx;
      4'h2: f = 0;
      4'h3: f = 1'bx;
      4'h4: f = 1;
      4'h5: f = 1'bx;
      4'h6: f = 1;
      4'h7: f = 0;
      4'h8: f = 0;
      4'h9: f = 0;
      4'ha: f = 1'bx;
      4'hb: f = 1;
      4'hc: f = 1;
      4'hd: f = 1'bx;
      4'he: f = 1;
      4'hf: f = 1'bx;
    endcase
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob117_circuit9
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  a
 - output q (3 bits)

The module implements a sequential circuit. Read the simulation waveforms
to determine what the circuit does, then implement it.

  time  clk a   q
  0ns   0   1   x
  5ns   1   1   4
  10ns  0   1   4
  15ns  1   1   4
  20ns  0   1   4
  25ns  1   1   4
  30ns  0   1   4
  35ns  1   1   4
  40ns  0   1   4
  45ns  1   0   4
  50ns  0   0   4
  55ns  1   0   5
  60ns  0   0   5
  65ns  1   0   6
  70ns  0   0   6
  75ns  1   0   0
  80ns  0   0   0
  85ns  1   0   1
  90ns  0   0   1
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input a,
  output reg [2:0] q
);

  always @(posedge clk)
    if (a)
      q <= 4;
    else if (q == 6)
      q <= 0;
    else
      q <= q + 1'b1;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob118_history_shift
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  predict_valid
 - input  predict_taken
 - input  train_mispredicted
 - input  train_taken
 - input  train_history   (32 bits)
 - output predict_history (32 bits)

The module should implement a 32-bit global history shift register,
including support for rolling back state in response to a pipeline flush
caused by a branch misprediction. When a branch prediction is made
(predict_valid = 1), shift in predict_taken from the LSB side to update
the branch history for the predicted branch. (predict_history[0] is the
direction of the youngest branch.) When a branch misprediction occurs
(train_mispredicted = 1), load the branch history register with the
history after the completion of the mispredicted branch. This is the
history before the mispredicted branch (train_history) concatenated with
the actual result of the branch (train_taken). If both a prediction and
misprediction occur at the same time, the misprediction takes precedence,
because the pipeline flush will also flush out the branch that is
currently making a prediction. predict_history is the value of the branch
history register. areset is a positive edge triggered asynchronous reset
that resets the history counter to zero. Assume all sequential logic is
triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input predict_valid,
  input predict_taken,
  output logic [31:0] predict_history,

  input train_mispredicted,
  input train_taken,
  input [31:0] train_history
);

  always@(posedge clk, posedge areset)
    if (areset) begin
      predict_history = 0;
        end  else begin
      if (train_mispredicted)
        predict_history <= {train_history, train_taken};
      else if (predict_valid)
        predict_history <= {predict_history, predict_taken};
    end

endmodule
```

#### model output
```verilog

```
---

### Prob119_fsm3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  in
 - output out

The module should implement a Moore state machine with the following
state transition table with one input, one output, and four states.
Implement this state machine. Include a positive edge triggered
asynchronous reset that resets the FSM to state A. Assume all sequential
logic is triggered on the positive edge of the clock.

  state | next state in=0, next state in=1 | output
  A     | A, B                             | 0
  B     | C, B                             | 0
  C     | A, D                             | 0
  D     | C, B                             | 1
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input in,
  input areset,
  output out
);

  parameter A=0, B=1, C=2, D=3;
  reg [1:0] state;
  reg [1:0] next;

  always_comb begin
  case (state)
    A: next = in ? B : A;
    B: next = in ? B : C;
    C: next = in ? D : A;
    D: next = in ? B : C;
  endcase
  end

  always @(posedge clk, posedge areset) begin
    if (areset) state <= A;
      else state <= next;
  end

  assign out = (state==D);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire areset,
    input  wire in,
    output reg  out
);
    // State encoding
    localparam [1:0] A = 2'b00,
                     B = 2'b01,
                     C = 2'b10,
                     D = 2'b11;

    reg [1:0] state, next_state;

    // Next‑state combinational logic
    always @* begin
        case (state)
            A: next_state = (in) ? B : A;
            B: next_state = (in) ? B : C;
            C: next_state = (in) ? D : A;
            D: next_state = (in) ? B : C;
            default: next_state = A;
        endcase
    end

    // State register with positive‑edge‑triggered asynchronous reset
    always @(posedge clk or posedge areset) begin
        if (areset)
            state <= A;
        else
            state <= next_state;
    end

    // Moore output logic
    always @* begin
        case (state)
            D: out = 1'b1;
            default: out = 1'b0;
        endcase
    end
endmodule
```
---

### Prob120_fsm3s
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  in
 - output out

The module should implement a Moore state machine with the following
state transition table with one input, one output, and four states.
Include a synchronous active high reset that resets the FSM to state A.
Assume all sequential logic is triggered on the positive edge of the
clock.

  State | Next state in=0, Next state in=1 | Output
  A     | A, B                             | 0
  B     | C, B                             | 0
  C     | A, D                             | 0
  D     | C, B                             | 1
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input in,
  input reset,
  output out
);

  parameter A=0, B=1, C=2, D=3;
  reg [1:0] state;
  reg [1:0] next;

  always_comb begin
    case (state)
      A: next = in ? B : A;
      B: next = in ? B : C;
      C: next = in ? D : A;
      D: next = in ? B : C;
    endcase
  end

  always @(posedge clk) begin
    if (reset) state <= A;
      else state <= next;
  end

  assign out = (state==D);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire in,
    output wire out
);
    // State encoding
    localparam A = 2'b00,
               B = 2'b01,
               C = 2'b10,
               D = 2'b11;

    reg [1:0] state;
    reg [1:0] next_state;

    // Next‑state combinational logic (Moore FSM)
    always @(*) begin
        case (state)
            A: next_state = in ? B : A;
            B: next_state = in ? B : C;
            C: next_state = in ? D : A;
            D: next_state = in ? B : C;
            default: next_state = A;
        endcase
    end

    // State register with synchronous active‑high reset
    always @(posedge clk) begin
        if (reset)
            state <= A;
        else
            state <= next_state;
    end

    // Output logic (Moore)
    assign out = (state == D);

endmodule
```
---

### Prob121_2014_q3bfsm
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  x
 - output z

The module should implement a finite-state machine with the
state-assigned table shown below. Reset should synchronous active high
reset the FSM to state 000. Assume all sequential logic is triggered on
the positive edge of the clock.

  Present state y[2:0] | Next state y[2:0] x=0, Next state y[2:0] x=1 | Output z
  000 | 000, 001 | 0
  001 | 001, 100 | 0
  010 | 010, 001 | 0
  011 | 001, 010 | 1
  100 | 011, 100 | 1
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input x,
  output reg z
);

  parameter A=0, B=1, C=2, D=3, E=4;
  reg [2:0] state, next;

  always @(posedge clk) begin
    if (reset) state <= A;
    else state <= next;
  end

  always_comb begin
    case (state)
      A: next = x ? B : A;
      B: next = x ? E : B;
      C: next = x ? B : C;
      D: next = x ? C : B;
      E: next = x ? E : D;
      default: next = 'x;
    endcase
  end

  assign z = (state == D) || (state == E);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire x,
    output wire z
);

    // State encoding
    reg [2:0] state;
    reg [2:0] next_state;

    // State register with synchronous active‑high reset
    always @(posedge clk) begin
        if (reset)
            state <= 3'b000;
        else
            state <= next_state;
    end

    // Next‑state logic
    always @(*) begin
        case (state)
            3'b000: next_state = x ? 3'b001 : 3'b000;
            3'b001: next_state = x ? 3'b100 : 3'b001;
            3'b010: next_state = x ? 3'b001 : 3'b010;
            3'b011: next_state = x ? 3'b010 : 3'b001;
            3'b100: next_state = x ? 3'b100 : 3'b011;
            default: next_state = 3'b000;
        endcase
    end

    // Output logic
    assign z = (state == 3'b011) || (state == 3'b100);

endmodule
```
---

### Prob122_kmap4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - output out

The module should implement the Karnaugh map below.

             ab
  cd   00  01  11  10
  00 | 0 | 1 | 0 | 1 |
  01 | 1 | 0 | 1 | 0 |
  11 | 0 | 1 | 0 | 1 |
  10 | 1 | 0 | 1 | 0 |
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output reg out
);

  always @(*) begin
    case({a,b,c,d})
      4'h0: out = 0;
      4'h1: out = 1;
      4'h3: out = 0;
      4'h2: out = 1;
      4'h4: out = 1;
      4'h5: out = 0;
      4'h7: out = 1;
      4'h6: out = 0;
      4'hc: out = 0;
      4'hd: out = 1;
      4'hf: out = 0;
      4'he: out = 1;
      4'h8: out = 1;
      4'h9: out = 0;
      4'hb: out = 1;
      4'ha: out = 0;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire a,
    input  wire b,
    input  wire c,
    input  wire d,
    output wire out
);
    // Output is the XOR of all four inputs (parity function)
    assign out = a ^ b ^ c ^ d;
endmodule
```
---

### Prob123_bugs_addsubz
<details>
<summary>prompt</summary>

```text
Consider the following adder-subtractor with a zero flag:

  synthesis verilog_input_version verilog_2001
  module TopModule (
      input do_sub,
      input [7:0] a,
      input [7:0] b,
      output reg [7:0] out,
      output reg result_is_zero
  );

      always @(*) begin
          case (do_sub)
            0: out = a+b;
            1: out = a-b;
          endcase

          if (~out)
              result_is_zero = 1;
      end

  endmodule

Unfortunately, this module has a bug. Implement a new version of this
module that fixes the bug.
```
</details>

#### reference
```verilog
module RefModule (
  input do_sub,
  input [7:0] a,
  input [7:0] b,
  output reg [7:0] out,
  output reg result_is_zero
);

  always @(*) begin
    case (do_sub)
      0: out = a + b;
      1: out = a - b;
    endcase
    result_is_zero = (out == 0);
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire       do_sub,
    input  wire [7:0] a,
    input  wire [7:0] b,
    output reg  [7:0] out,
    output reg        result_is_zero
);
    always @* begin
        // Perform addition or subtraction based on do_sub
        if (do_sub)
            out = a - b;
        else
            out = a + b;

        // Zero flag is asserted when the result is exactly zero
        result_is_zero = (out == 8'b0);
    end
endmodule
```
---

### Prob124_rule110
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  load
 - input  data (512 bits)
 - output q    (512 bits)

The module should implement Rule 110, a one-dimensional cellular
automaton with interesting properties (such as being Turing-complete).
There is a one-dimensional array of cells (on or off). At each time step,
the state of each cell changes. In Rule 110, the next state of each cell
depends only on itself and its two neighbours, according to the following
table:

  Left[i+1] | Center[i] | Right[i-1] | Center's next state 
  1         | 1         | 1          | 0
  1         | 1         | 0          | 1
  1         | 0         | 1          | 1
  1         | 0         | 0          | 0
  0         | 1         | 1          | 1
  0         | 1         | 0          | 1
  0         | 0         | 1          | 1
  0         | 0         | 0          | 0

In this circuit, create a 512-cell system (q[511:0]), and advance by one
time step each clock cycle. The synchronous active high load input
indicates the state of the system should be loaded with data[511:0].
Assume the boundaries (q[-1] and q[512], if they existed) are both zero
(off). Assume all sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input load,
  input [511:0] data,
  output reg [511:0] q
);

  always @(posedge clk) begin
    if (load)
      q <= data;
    else begin
      q <=
      ~((q[$bits(q)-1:1] & q[$bits(q)-1:0] & {q[$bits(q)-2:0], 1'b0}) |
      (~q[$bits(q)-1:1] & ~q[$bits(q)-1:0] & ~{q[$bits(q)-2:0], 1'b0}) |
      (q[$bits(q)-1:1] & ~q[$bits(q)-1:0] & ~{q[$bits(q)-2:0], 1'b0}) )
      ;
    end
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob125_kmap3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a
 - input  b
 - input  c
 - input  d
 - output out

The module should implement the Karnaugh map below. d is don't-care,
which means you may choose to output whatever value is convenient.

              ab
   cd   01  00  10  11
   00 | d | 0 | 1 | 1 |
   01 | 0 | 0 | d | d |
   11 | 0 | 1 | 1 | 1 |
   10 | 0 | 1 | 1 | 1 |
```
</details>

#### reference
```verilog
module RefModule (
  input a,
  input b,
  input c,
  input d,
  output reg out
);

  always @(*) begin
    case({a,b,c,d})
      4'h0: out = 0;
      4'h1: out = 0;
      4'h3: out = 1;
      4'h2: out = 1;
      4'h4: out = 1'bx;
      4'h5: out = 0;
      4'h7: out = 0;
      4'h6: out = 0;
      4'hc: out = 1;
      4'hd: out = 1'bx;
      4'hf: out = 1;
      4'he: out = 1;
      4'h8: out = 1;
      4'h9: out = 1'bx;
      4'hb: out = 1;
      4'ha: out = 1;
    endcase
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob126_circuit6
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a ( 3 bits)
 - output q (16 bits)

The module should implement a combinational circuit. Read the simulation
waveforms to determine what the circuit does, then implement it.

  time  a     q
  0ns   x     x
  5ns   x     x
  10ns  x     x
  15ns  0  1232
  20ns  1  aee0
  25ns  2  27d4
  30ns  3  5a0e
  35ns  4  2066
  40ns  5  64ce
  45ns  6  c526
  50ns  7  2f19
  55ns  0  1232
  60ns  1  aee0
  65ns  2  27d4
  70ns  4  2066
  75ns  1  aee0
  80ns  1  aee0
  85ns  3  5a0e
  90ns  5  64ce
```
</details>

#### reference
```verilog
module RefModule (
  input [2:0] a,
  output reg [15:0] q
);

  always @(*)
    case (a)
      0: q = 4658;
      1: q = 44768;
      2: q = 10196;
      3: q = 23054;
      4: q = 8294;
      5: q = 25806;
      6: q = 50470;
      7: q = 12057;
    endcase

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [2:0] a,
    output reg  [15:0] q
);
always @* begin
    case (a)
        3'd0: q = 16'h1232;
        3'd1: q = 16'hAEE0;
        3'd2: q = 16'h27D4;
        3'd3: q = 16'h5A0E;
        3'd4: q = 16'h2066;
        3'd5: q = 16'h64CE;
        3'd6: q = 16'hC526;
        3'd7: q = 16'h2F19;
        default: q = 16'hXXXX;
    endcase
end
endmodule
```
---

### Prob127_lemmings1
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  bump_left
 - input  bump_right
 - output walk_left
 - output walk_right

The module should implement a simple game called Lemmings which involves
critters with fairly simple brains. So simple that we are going to model
it using a finite state machine. In the Lemmings' 2D world, Lemmings can
be in one of two states: walking left (walk_left is 1) or walking right
(walk_right is 1). It will switch directions if it hits an obstacle. In
particular, if a Lemming is bumped on the left (by receiving a 1 on
bump_left), it will walk right. If it's bumped on the right (by receiving
a 1 on bump_right), it will walk left. If it's bumped on both sides at
the same time, it will still switch directions.

The module should implement a Moore state machine with two states, two
inputs, and one output (internal to the module) that models this
behaviour. areset is positive edge triggered asynchronous resetting the
Lemming machine to walk left. Assume all sequential logic is triggered on
the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input bump_left,
  input bump_right,
  output walk_left,
  output walk_right
);

  parameter WL=0, WR=1;
  reg state;
  reg next;

  always_comb begin
    case (state)
      WL: next = bump_left ? WR : WL;
      WR: next = bump_right ? WL: WR;
    endcase
  end

  always @(posedge clk, posedge areset) begin
    if (areset) state <= WL;
      else state <= next;
  end

  assign walk_left = (state==WL);
  assign walk_right = (state==WR);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire areset,
    input  wire bump_left,
    input  wire bump_right,
    output reg  walk_left,
    output reg  walk_right
);

    // State encoding: 0 = walking left, 1 = walking right
    reg state;
    reg next_state;

    // Asynchronous positive‑edge reset, synchronous state update
    always @(posedge clk or posedge areset) begin
        if (areset)
            state <= 1'b0;          // reset to walking left
        else
            state <= next_state;
    end

    // Next‑state logic (Moore FSM)
    always @* begin
        if (bump_left || bump_right)
            next_state = ~state;    // switch direction on any bump
        else
            next_state = state;     // retain current direction
    end

    // Output logic (Moore: outputs depend only on current state)
    always @* begin
        walk_left  = (state == 1'b0);
        walk_right = (state == 1'b1);
    end

endmodule
```
---

### Prob128_fsm_ps2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  in (8 bits)
 - output done

The PS/2 mouse protocol sends messages that are three bytes long.
However, within a continuous byte stream, it's not obvious where messages
start and end. The only indication is that the first byte of each three
byte message always has in[3]=1 (but in[3] of the other two bytes may be
1 or 0 depending on data).

The module should implement a finite state machine that will search for
message boundaries when given an input byte stream. The algorithm we'll
use is to discard bytes until we see one with in[3]=1. We then assume
that this is byte 1 of a message, and signal the receipt of a message
once all 3 bytes have been received (done). The FSM should signal done in
the cycle immediately after the third byte of each message was
successfully received. Reset should be active high synchronous. Assume
all sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] in,
  input reset,
  output done
);

  parameter BYTE1=0, BYTE2=1, BYTE3=2, DONE=3;
  reg [1:0] state;
  reg [1:0] next;

  wire in3 = in[3];

  always_comb begin
    case (state)
      BYTE1: next = in3 ? BYTE2 : BYTE1;
      BYTE2: next = BYTE3;
      BYTE3: next = DONE;
      DONE: next = in3 ? BYTE2 : BYTE1;
    endcase
  end

  always @(posedge clk) begin
    if (reset) state <= BYTE1;
      else state <= next;
  end

  assign done = (state==DONE);

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob129_ece241_2013_q8
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  aresetn
 - input  x
 - output z

The module should implement a Mealy-type finite state machine that
recognizes the sequence "101" on an input signal named x. Your FSM should
have an output signal, z, that is asserted to logic-1 when the "101"
sequence is detected. Your FSM should also have a negative edge triggered
asynchronous reset. You may only have 3 states in your state machine.
Your FSM should recognize overlapping sequences. Assume all sequential
logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input aresetn,
  input x,
  output reg z
);

  parameter S=0, S1=1, S10=2;
  reg[1:0] state, next;

  always@(posedge clk, negedge aresetn)
    if (!aresetn)
      state <= S;
    else
      state <= next;

  always_comb begin
    case (state)
      S: next = x ? S1 : S;
      S1: next = x ? S1 : S10;
      S10: next = x ? S1 : S;
      default: next = 'x;
    endcase
  end

  always_comb begin
    case (state)
      S: z = 0;
      S1: z = 0;
      S10: z = x;
      default: z = 'x;
    endcase
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire aresetn,
    input  wire x,
    output reg  z
);
    // State encoding (3 states)
    localparam S0 = 2'd0,
               S1 = 2'd1,
               S2 = 2'd2;

    reg [1:0] state, next_state;

    // Sequential logic with negative‑edge asynchronous reset
    always @(posedge clk or negedge aresetn) begin
        if (!aresetn)
            state <= S0;
        else
            state <= next_state;
    end

    // Mealy next‑state and output logic
    always @(*) begin
        // defaults
        next_state = state;
        z = 1'b0;

        case (state)
            S0: begin
                if (x)
                    next_state = S1;
                else
                    next_state = S0;
            end
            S1: begin
                if (x)
                    next_state = S1;   // stay, possible overlapping start
                else
                    next_state = S2;
            end
            S2: begin
                if (x) begin
                    z = 1'b1;          // sequence 101 detected
                    next_state = S1;   // overlapping: last '1' is start of new pattern
                end else begin
                    next_state = S0;
                end
            end
            default: begin
                next_state = S0;
                z = 1'b0;
            end
        endcase
    end
endmodule
```
---

### Prob130_circuit5
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  a (4 bits)
 - input  b (4 bits)
 - input  c (4 bits)
 - input  d (4 bits)
 - input  e (4 bits)
 - output q (4 bits)

The module should implement a combinational circuit. Read the simulation
waveforms to determine what the circuit does, then implement it.

  time  a  b  c  d  e  q
  0ns   x  x  x  x  x  x
  5ns   x  x  x  x  x  x
  10ns  x  x  x  x  x  x
  15ns  a  b  0  d  e  b
  20ns  a  b  1  d  e  e
  25ns  a  b  2  d  e  a
  30ns  a  b  3  d  e  d
  35ns  a  b  4  d  e  f
  40ns  a  b  5  d  e  f
  45ns  a  b  6  d  e  f
  50ns  a  b  7  d  e  f
  55ns  a  b  8  d  e  f
  60ns  a  b  9  d  e  f
  65ns  a  b  a  d  e  f
  70ns  a  b  b  d  e  f
  75ns  a  b  c  d  e  f
  80ns  a  b  d  d  e  f
  85ns  a  b  e  d  e  f
  90ns  a  b  f  d  e  f
```
</details>

#### reference
```verilog
module RefModule (
  input [3:0] a,
  input [3:0] b,
  input [3:0] c,
  input [3:0] d,
  input [3:0] e,
  output reg [3:0] q
);

  always @(*)
    case (c)
      0: q = b;
      1: q = e;
      2: q = a;
      3: q = d;
      default: q = 4'hf;
    endcase

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire [3:0] a,
    input  wire [3:0] b,
    input  wire [3:0] c,
    input  wire [3:0] d,
    input  wire [3:0] e,
    output reg  [3:0] q
);

    // Combinational logic: select output based on selector c
    always @* begin
        case (c)
            4'd0: q = b;   // selector 0 -> b
            4'd1: q = e;   // selector 1 -> e
            4'd2: q = a;   // selector 2 -> a
            4'd3: q = d;   // selector 3 -> d
            default: q = 4'b1111; // selector >=4 -> constant f (1111)
        endcase
    end

endmodule
```
---

### Prob131_mt2015_q4
<details>
<summary>prompt</summary>

```text
Module A implements the boolean function z = (x^y) & x.

Module B can be described by the following simulation waveform:

  time  x  y  z
  0ns   0  0  1
  5ns   0  0  1
  10ns  0  0  1
  15ns  0  0  1
  20ns  0  0  1
  25ns  1  0  0
  30ns  1  0  0
  35ns  0  1  0
  40ns  0  1  0
  45ns  1  1  1
  50ns  1  1  1
  55ns  0  0  1
  60ns  0  1  0
  65ns  0  1  0
  70ns  1  1  1
  75ns  0  1  0
  80ns  0  1  0
  85ns  0  1  0
  90ns  1  0  0

Now consider a top-level module with the following interface:

 - input  x
 - input  y
 - output z

The module is implemented with two A submodules and two B submodules. The
first input of all four submodules is connect to input 'x', and the
second input of all four submodules is connected to 'y'. The output of
the first A submodule is connected to a two-input OR, along with the
output of the first B submodule. The second pair of A and B submodules is
similarly connected to an AND gate. The output of the OR and the AND is
connected to an XOR, whose output is 'z'.
```
</details>

#### reference
```verilog
module RefModule (
  input x,
  input y,
  output z
);

  assign z = x|~y;

endmodule
```

#### model output
```verilog

```
---

### Prob132_always_if2
<details>
<summary>prompt</summary>

```text
Consider the following adder-subtractor with a zero flag:

  module TopModule (
      input      cpu_overheated,
      output reg shut_off_computer,
      input      arrived,
      input      gas_tank_empty,
      output reg keep_driving
  );

      always @(*) begin
          if (cpu_overheated)
             shut_off_computer = 1;
      end

      always @(*) begin
          if (~arrived)
             keep_driving = ~gas_tank_empty;
      end

  endmodule

Unfortunately, this module has a bug. Implement a new version of this
module that fixes the bug.
```
</details>

#### reference
```verilog
module RefModule (
  input cpu_overheated,
  output reg shut_off_computer,
  input arrived,
  input gas_tank_empty,
  output reg keep_driving
);

  always @(*) begin
    if (cpu_overheated)
      shut_off_computer = 1;
    else
      shut_off_computer = 0;
  end

  always @(*) begin
    if (~arrived)
      keep_driving = !gas_tank_empty;
    else
      keep_driving = 1'b0;
  end

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire cpu_overheated,
    output reg  shut_off_computer,
    input  wire arrived,
    input  wire gas_tank_empty,
    output reg  keep_driving
);
    // Combinational logic with default assignments to avoid latches
    always @* begin
        shut_off_computer = 1'b0;
        if (cpu_overheated)
            shut_off_computer = 1'b1;
    end

    always @* begin
        keep_driving = 1'b0;
        if (arrived)
            keep_driving = gas_tank_empty;
    end
endmodule
```
---

### Prob133_2014_q3fsm
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  s
 - input  w
 - output z

The module should implement a finite state machine with inputs s and w.
Assume that the FSM begins in a reset state called A, as depicted below.
The FSM remains in state A as long as s = 0, and it moves to state B when
s = 1. Once in state B the FSM examines the value of the input w in the
next three clock cycles. If w = 1 in exactly two of these clock cycles,
then the FSM has to set an output z to 1 in the following clock cycle.
Otherwise z has to be 0. The FSM continues checking w for the next three
clock cycles, and so on. Use as few states as possible. Note that the s
input is used only in state A, so you need to consider just the w input.
Assume reset is active high synchronous. Assume all sequential logic is
triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input s,
  input w,
  output reg z
);

  parameter A=0, B=1, C=2, S10=3, S11=4, S20=5, S21=6, S22=7;
  reg [2:0] state, next;

  always @(posedge clk) begin
    if (reset) state <= A;
    else state <= next;
  end

  always_comb begin
    case (state)
      A: next = s ? B : A;
      B: next = w ? S11 : S10;
      C: next = w ? S11 : S10;
      S10: next = w ? S21 : S20;
      S11: next = w ? S22 : S21;
      S20: next = B;
      S21: next = w ? C : B;
      S22: next = w ? B : C;
      default: next = 'x;
    endcase
  end

  assign z = (state == C);

endmodule
```

#### model output
```verilog

```
---

### Prob134_2014_q3c
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  x
 - input  y (3 bits)
 - output Y0
 - output z

The module should implement the output logic and next state logic for an
FSM using the table shown below. Note that the output Y0 is Y[0] of the
next state signal.

   Present state input y[2:0] | Next state Y[2:0] when x=0, Next state Y[2:0] when x=1 | Output z
   000 | 000, 001 | 0
   001 | 001, 100 | 0
   010 | 010, 001 | 0
   011 | 001, 010 | 1
   100 | 011, 100 | 1
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input x,
  input [2:0] y,
  output reg Y0,
  output reg z
);

  always_comb begin
    case ({y[2:0], x})
      4'h0: Y0 = 0;
      4'h1: Y0 = 1;
      4'h2: Y0 = 1;
      4'h3: Y0 = 0;
      4'h4: Y0 = 0;
      4'h5: Y0 = 1;
      4'h6: Y0 = 1;
      4'h7: Y0 = 0;
      4'h8: Y0 = 1;
      4'h9: Y0 = 0;
      default: Y0 = 1'bx;
    endcase

    case (y[2:0])
      3'h0: z = 0;
      3'h1: z = 0;
      3'h2: z = 0;
      3'h3: z = 1;
      3'h4: z = 1;
      default: z = 1'bx;
    endcase
  end

endmodule
```

#### model output
```verilog

```
---

### Prob135_m2014_q6b
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  y (3 bits)
 - input  w
 - output Y1

The module should implement the state machine shown below:

  A (0) --0--> B
  A (0) --1--> A
  B (0) --0--> C
  B (0) --1--> D
  C (0) --0--> E
  C (0) --1--> D
  D (0) --0--> F
  D (0) --1--> A
  E (1) --0--> E
  E (1) --1--> D
  F (1) --0--> C
  F (1) --1--> D

The FSM should be implemented using three flip-flops and state codes
y = 000, 001, ..., 101 for states A, B, ..., F, respectively. Implement
just the next-state logic for y[1]. The output Y1 is y[1].
```
</details>

#### reference
```verilog
module RefModule (
  input [2:0] y,
  input w,
  output reg Y1
);

  always_comb begin
    case ({y, w})
      4'h0: Y1 = 1'b0;
      4'h1: Y1 = 1'b0;
      4'h2: Y1 = 1'b1;
      4'h3: Y1 = 1'b1;
      4'h4: Y1 = 1'b0;
      4'h5: Y1 = 1'b1;
      4'h6: Y1 = 1'b0;
      4'h7: Y1 = 1'b0;
      4'h8: Y1 = 1'b0;
      4'h9: Y1 = 1'b1;
      4'ha: Y1 = 1'b1;
      4'hb: Y1 = 1'b1;
      default: Y1 = 1'bx;
    endcase
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob136_m2014_q6
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  w
 - output z

The module should implement the state machine shown below:

  A (0) --0--> B
  A (0) --1--> A
  B (0) --0--> C
  B (0) --1--> D
  C (0) --0--> E
  C (0) --1--> D
  D (0) --0--> F
  D (0) --1--> A
  E (1) --0--> E
  E (1) --1--> D
  F (1) --0--> C
  F (1) --1--> D

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input w,
  output z
);

  parameter A=0, B=1, C=2, D=3, E=4, F=5;
  reg [2:0] state, next;

  always @(posedge clk)
    if (reset)
      state <= A;
    else
      state <= next;

  always_comb begin
    case(state)
      A: next = w ? A : B;
      B: next = w ? D : C;
      C: next = w ? D : E;
      D: next = w ? A : F;
      E: next = w ? D : E;
      F: next = w ? D : C;
      default: next = 'x;
    endcase
  end

  assign z = (state == E || state == F);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire w,
    output reg  z
);

    // State encoding
    localparam A = 3'd0,
               B = 3'd1,
               C = 3'd2,
               D = 3'd3,
               E = 3'd4,
               F = 3'd5;

    reg [2:0] state, next_state;

    // State register
    always @(posedge clk) begin
        if (reset)
            state <= A;
        else
            state <= next_state;
    end

    // Next-state logic
    always @* begin
        case (state)
            A: begin
                if (w == 1'b0) next_state = B;
                else           next_state = A;
            end
            B: begin
                if (w == 1'b0) next_state = C;
                else           next_state = D;
            end
            C: begin
                if (w == 1'b0) next_state = E;
                else           next_state = D;
            end
            D: begin
                if (w == 1'b0) next_state = F;
                else           next_state = A;
            end
            E: begin
                if (w == 1'b0) next_state = E;
                else           next_state = D;
            end
            F: begin
                if (w == 1'b0) next_state = C;
                else           next_state = D;
            end
            default: next_state = A;
        endcase
    end

    // Output logic (Moore)
    always @* begin
        case (state)
            E, F: z = 1'b1;
            default: z = 1'b0;
        endcase
    end

endmodule
```
---

### Prob137_fsm_serial
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  in
 - output done

In many (older) serial communications protocols, each data byte is sent
along with a start bit and a stop bit, to help the receiver delimit bytes
from the stream of bits. One common scheme is to use one start bit (0), 8
data bits, and 1 stop bit (1). The line is also at logic 1 when nothing
is being transmitted (idle).

Implement a finite state machine that will identify when bytes have been
correctly received when given a stream of bits. It needs to identify the
start bit, wait for all 8 data bits, then verify that the stop bit was
correct. If the stop bit does not appear when expected, the FSM must wait
until it finds a stop bit before attempting to receive the next byte.
Include a active-high synchronous reset. Note that the serial protocol
sends the least significant bit first.

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input in,
  input reset,
  output done
);

  parameter B0=0, B1=1, B2=2, B3=3, B4=4, B5=5, B6=6, B7=7, START=8, STOP=9, DONE=10, ERR=11;
  reg [3:0] state;
  reg [3:0] next;

  always_comb begin
    case (state)
      START: next = in ? START : B0;  // start bit is 0
      B0: next = B1;
      B1: next = B2;
      B2: next = B3;
      B3: next = B4;
      B4: next = B5;
      B5: next = B6;
      B6: next = B7;
      B7: next = STOP;
      STOP: next = in ? DONE : ERR;  // stop bit is 1. Idle state is 1.
      DONE: next = in ? START : B0;
      ERR: next = in ? START : ERR;
    endcase
  end

  always @(posedge clk) begin
    if (reset) state <= START;
      else state <= next;
  end

  assign done = (state==DONE);

endmodule
```

#### model output
```verilog

```
---

### Prob138_2012_q2fsm
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  w
 - output z

The module should implement the state machine shown below:

  A (0) --1--> B
  A (0) --0--> A
  B (0) --1--> C
  B (0) --0--> D
  C (0) --1--> E
  C (0) --0--> D
  D (0) --1--> F
  D (0) --0--> A
  E (1) --1--> E
  E (1) --0--> D
  F (1) --1--> C
  F (1) --0--> D

Reset resets into state A and is synchronous active-high. Assume all
sequential logic is triggered on the positive edge of the clock.

Use separate always blocks for the state table and the state flip-flops.
Describe the FSM output, which is called _z_, using either continuous
assignment statement(s) or an always block (at your discretion). Assign
any state codes that you wish to use.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input w,
  output z
);

  parameter A=0,B=1,C=2,D=3,E=4,F=5;
  reg[2:0] state, next;

  always @(posedge clk)
    if (reset) state <= A;
    else state <= next;

  always_comb begin
    case (state)
      A: next = w ? B : A;
      B: next = w ? C : D;
      C: next = w ? E : D;
      D: next = w ? F : A;
      E: next = w ? E : D;
      F: next = w ? C : D;
      default: next = 'x;
    endcase
  end

  assign z = (state == E) || (state == F);

endmodule
```

#### model output
```verilog
module TopModule (
    input  wire clk,
    input  wire reset,
    input  wire w,
    output wire z
);

    // State encoding
    localparam A = 3'd0,
               B = 3'd1,
               C = 3'd2,
               D = 3'd3,
               E = 3'd4,
               F = 3'd5;

    // State registers
    reg [2:0] state, next_state;

    // State flip‑flops (synchronous reset)
    always @(posedge clk) begin
        if (reset)
            state <= A;
        else
            state <= next_state;
    end

    // Next‑state combinational logic
    always @(*) begin
        case (state)
            A: next_state = w ? B : A;
            B: next_state = w ? C : D;
            C: next_state = w ? E : D;
            D: next_state = w ? F : A;
            E: next_state = w ? E : D;
            F: next_state = w ? C : D;
            default: next_state = A;
        endcase
    end

    // Output logic (Moore)
    assign z = (state == E) || (state == F);

endmodule
```
---

### Prob139_2013_q2bfsm
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  resetn
 - input  x
 - input  y
 - output f
 - output g

Consider a finite state machine that is used to control some type of
motor. The FSM has inputs x and y, which come from the motor, and
produces outputs f and g, which control the motor. There is also a clock
input called clk and a reset input (synchronous, active low) called
resetn. The FSM has to work as follows. As long as the reset input is
asserted, the FSM stays in a beginning state, called state A. When the
reset signal is de-asserted, then after the next clock edge the FSM has
to set the output f to 1 for one clock cycle. Then, the FSM has to
monitor the x input. When x has produced the values 1, 0, 1 in three
successive clock cycles, then g should be set to 1 on the following clock
cycle. While maintaining g = 1 the FSM has to monitor the y input. If y
has the value 1 within at most two clock cycles, then the FSM should
maintain g = 1 permanently (that is, until reset). But if y does not
become 1 within two clock cycles, then the FSM should set g = 0
permanently (until reset). Assume all sequential logic is triggered on
the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input resetn,
  input x,
  input y,
  output f,
  output g
);

  parameter A=0, B=1, S0=2, S1=3, S10=4, G1=5, G2=6, P0=7, P1=8;
  reg [3:0] state, next;

  always @(posedge clk) begin
    if (~resetn) state <= A;
    else state <= next;
  end

  always_comb begin
    case (state)
      A: next = B;
      B: next = S0;
      S0: next = x ? S1 : S0;
      S1: next = x ? S1 : S10;
      S10: next = x? G1 : S0;
      G1: next = y ? P1 : G2;
      G2: next = y ? P1 : P0;
      P0: next = P0;
      P1: next = P1;
      default: next = 'x;
    endcase
  end

  assign f = (state == B);
  assign g = (state == G1) || (state == G2) || (state == P1);

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob140_fsm_hdlc
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  in
 - output disc
 - output flag
 - output err

Synchronous HDLC framing involves decoding a continuous bit stream of
data to look for bit patterns that indicate the beginning and end of
frames (packets). Seeing exactly 6 consecutive 1s (i.e., 01111110) is a
"flag" that indicate frame boundaries. To avoid the data stream from
accidentally containing "flags", the sender inserts a zero after every 5
consecutive 1s which the receiver must detect and discard. We also need
to signal an error if there are 7 or more consecutive 1s. Create a
Moore-type finite state machine to recognize these three sequences:

  (1) 0111110: Signal a bit needs to be discarded (disc).
  (2) 01111110: Flag the beginning/end of a frame (flag).
  (3) 01111111...: Error (7 or more 1s) (err).

When the FSM is reset, it should be in a state that behaves as though the
previous input were 0. The reset signal is active high synchronous. The
output signals should be asserted for a complete cycle beginning on the
clock cycle after the condition occurs. Assume all sequential
logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input in,
  output disc,
  output flag,
  output err
);

  parameter [3:0] S0=0, S1=1, S2=2, S3=3, S4=4, S5=5, S6=6, SERR=7, SDISC=8, SFLAG=9;
  reg [3:0] state, next;

  assign disc = state == SDISC;
  assign flag = state == SFLAG;
  assign err = state == SERR;

  always @(posedge clk) begin
    case (state)
      S0: state <= in ? S1 : S0;
      S1: state <= in ? S2 : S0;
      S2: state <= in ? S3 : S0;
      S3: state <= in ? S4 : S0;
      S4: state <= in ? S5 : S0;
      S5: state <= in ? S6 : SDISC;
      S6: state <= in ? SERR : SFLAG;
      SERR: state <= in ? SERR : S0;
      SFLAG: state <= in ? S1 : S0;
      SDISC: state <= in ? S1 : S0;
      default: state <= 'x;
    endcase

    if (reset) state <= S0;
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob141_count_clock
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  ena
 - output pm
 - output hh (8 bits)
 - output mm (8 bits)
 - output ss (8 bits)

Create a set of counters suitable for use as a 12-hour clock (with am/pm
indicator). Your counters are clocked by a fast-running clk, with a pulse
on ena whenever your clock should increment (i.e., once per second, while
"clk" is much faster than once per second). The signal "pm" is asserted
if the clock is PM, or is otherwise AM. hh, mm, and ss are two BCD
(Binary- Coded Decimal) digits each for hours (01-12), minutes (00-59),
and seconds (00-59). Reset is the active high synchronous signal that
resets the clock to "12:00 AM." Reset has higher priority than enable and
can occur even when not enabled. Assume all sequential logic is triggered
on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input ena,
  output reg pm,
  output reg [7:0] hh,
  output reg [7:0] mm,
  output reg [7:0] ss
);

  wire [6:0] enable = {
    {hh[7:0],mm[7:0],ss[7:0]}==24'h115959,
    {hh[3:0],mm[7:0],ss[7:0]}==20'h95959,
    {mm[7:0],ss[7:0]}==16'h5959,
    {mm[3:0],ss[7:0]}==12'h959,
    ss[7:0]==8'h59,
    ss[3:0] == 4'h9,
    1'b1};

  always @(posedge clk)
    if (reset)
      {pm,hh,mm,ss} <= 25'h0120000;
    else if (ena) begin
      if (enable[0] && ss[3:0] == 9) ss[3:0] <= 0;
      else if (enable[0]) ss[3:0] <= ss[3:0] + 1;

      if (enable[1] && ss[7:4] == 4'h5) ss[7:4] <= 0;
      else if (enable[1]) ss[7:4] <= ss[7:4] + 1;

      if (enable[2] && mm[3:0] == 9) mm[3:0] <= 0;
      else if (enable[2]) mm[3:0] <= mm[3:0] + 1;

      if (enable[3] && mm[7:4] == 4'h5) mm[7:4] <= 0;
      else if (enable[3]) mm[7:4] <= mm[7:4] + 1;

      if (enable[4] && hh[3:0] == 4'h9) hh[3:0] <= 0;
      else if (enable[4]) hh[3:0] <= hh[3:0] + 1;

      if (enable[4] && hh[7:0] == 8'h12) hh[7:0] <= 8'h1;
      else if (enable[5]) hh[7:4] <= hh[7:4] + 1;

      if (enable[6]) pm <= ~pm;
    end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob142_lemmings2
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  bump_left
 - input  bump_right
 - input  ground
 - output walk_left
 - output walk_right
 - output aaah

The game Lemmings involves critters with fairly simple brains. So simple
that we are going to model it using a finite state machine. In the
Lemmings' 2D world, Lemmings can be in one of two states: walking left
(walk_left is 1) or walking right (walk_right is 1). It will switch
directions if it hits an obstacle. In particular, if a Lemming is bumped
on the left (by receiving a 1 on bump_left), it will walk right. If it's
bumped on the right (by receiving a 1 on bump_right), it will walk left.
If it's bumped on both sides at the same time, it will still switch
directions.

In addition to walking left and right and changing direction when bumped,
when ground=0, the Lemming will fall and say "aaah!". When the ground
reappears (ground=1), the Lemming will resume walking in the same
direction as before the fall. Being bumped while falling does not affect
the walking direction, and being bumped in the same cycle as ground
disappears (but not yet falling), or when the ground reappears while
still falling, also does not affect the walking direction.

Implement a Moore state machine that models this behaviour. areset is
positive edge triggered asynchronous reseting the Lemming machine to walk
left. Assume all sequential logic is triggered on the positive edge of
the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input bump_left,
  input bump_right,
  input ground,
  output walk_left,
  output walk_right,
  output aaah
);

  parameter WL=0, WR=1, FALLL=2, FALLR=3;
  reg [1:0] state;
  reg [1:0] next;

  always_comb begin
    case (state)
      WL: next = ground ? (bump_left ? WR : WL) : FALLL;
      WR: next = ground ? (bump_right ? WL: WR) : FALLR;
      FALLL: next = ground ? WL : FALLL;
      FALLR: next = ground ? WR : FALLR;
    endcase
  end

  always @(posedge clk, posedge areset) begin
    if (areset) state <= WL;
      else state <= next;
  end

  assign walk_left = (state==WL);
  assign walk_right = (state==WR);
  assign aaah = (state == FALLL) || (state == FALLR);

endmodule
```

#### model output
```verilog

```
---

### Prob143_fsm_onehot
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  in
 - input  state (10 bits)
 - output next_state (10 bits)
 - output out1
 - output out2

Given the follow state machine with 1 input and 2 outputs (the outputs
are given as "(out1, out2)"):

  S0 (0, 0) --0--> S0
  S0 (0, 0) --1--> S1
  S1 (0, 0) --0--> S0
  S1 (0, 0) --1--> S2
  S2 (0, 0) --0--> S0
  S2 (0, 0) --1--> S3
  S3 (0, 0) --0--> S0
  S3 (0, 0) --1--> S4
  S4 (0, 0) --0--> S0
  S4 (0, 0) --1--> S5
  S5 (0, 0) --0--> S8
  S5 (0, 0) --1--> S6
  S6 (0, 0) --0--> S9
  S6 (0, 0) --1--> S7
  S7 (0, 1) --0--> S0
  S7 (0, 1) --1--> S7
  S8 (1, 0) --0--> S0
  S8 (1, 0) --1--> S1
  S9 (1, 1) --0--> S0
  S9 (1, 1) --1--> S1

Suppose this state machine uses one-hot encoding, where state[0] through
state[9] correspond to the states S0 though S9, respectively. The outputs
are zero unless otherwise specified. The next_state[0] through next_state[9] 
correspond to the transition to next states S0 though S9. For example, The 
next_state[1] is set to 1 when the next state is S1 , otherwise, it is set to 0.

Here, the input state[9:0] can be a combinational of multiple states, and 
the TopModule is expected to response.
For example: 
When the state[9:0] = 10'b0000010100, state[4] == 1, and state[2] == 1, the 
states includes S4, and S2 states.

The module should implement the state transition logic and output logic
portions of the state machine (but not the state flip-flops). You are
given the current state in state[9:0] and must implement next_state[9:0] 
and the two outputs.
```
</details>

#### reference
```verilog
module RefModule (
  input in,
  input [9:0] state,
  output [9:0] next_state,
  output out1,
  output out2
);

  assign out1 = state[8] | state[9];
  assign out2 = state[7] | state[9];

  assign next_state[0] = !in && (|state[4:0] | state[7] | state[8] | state[9]);
  assign next_state[1] = in && (state[0] | state[8] | state[9]);
  assign next_state[2] = in && state[1];
  assign next_state[3] = in && state[2];
  assign next_state[4] = in && state[3];
  assign next_state[5] = in && state[4];
  assign next_state[6] = in && state[5];
  assign next_state[7] = in && (state[6] | state[7]);
  assign next_state[8] = !in && state[5];
  assign next_state[9] = !in && state[6];

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob144_conwaylife
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  load
 - input  data (256 bits)
 - output q (256 bits)

The module should implemement a "game" played on a two-dimensional grid
of cells, where each cell is either 1 (alive) or 0 (dead). At each time
step, each cell changes state depending on how many neighbours it has:

  (1) 0-1 neighbour: Cell becomes 0.
  (2) 2 neighbours: Cell state does not change.
  (3) 3 neighbours: Cell becomes 1.
  (4) 4+ neighbours: Cell becomes 0.

The game is formulated for an infinite grid. In this circuit, we will use
a 16x16 grid. To make things more interesting, we will use a 16x16
toroid, where the sides wrap around to the other side of the grid. For
example, the corner cell (0,0) has 8 neighbours: (15,1), (15,0), (15,15),
(0,1), (0,15), (1,1), (1,0), and (1,15). The 16x16 grid is represented by
a length 256 vector, where each row of 16 cells is represented by a
sub-vector: q[15:0] is row 0, q[31:16] is row 1, etc.

  (1) load: Loads data into q at the next clock edge, for loading initial
       state. Active high synchronous.
  (2) q: The 16x16 current state of the game, updated every clock cycle.

The game state should advance by one timestep every clock cycle. Assume
all sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input load,
  input [255:0] data,
  output reg [255:0] q
);

  logic [323:0] q_pad;
  always@(*) begin
    for (int i=0;i<16;i++)
      q_pad[18*(i+1)+1 +: 16] = q[16*i +: 16];
    q_pad[1 +: 16] = q[16*15 +: 16];
    q_pad[18*17+1 +: 16] = q[0 +: 16];

    for (int i=0; i<18; i++) begin
      q_pad[i*18] = q_pad[i*18+16];
      q_pad[i*18+17] = q_pad[i*18+1];
    end
  end

  always @(posedge clk) begin
    for (int i=0;i<16;i++)
    for (int j=0;j<16;j++) begin
      q[i*16+j] <=
        ((q_pad[(i+1)*18+j+1 -1+18] + q_pad[(i+1)*18+j+1 +18] + q_pad[(i+1)*18+j+1 +1+18] +
        q_pad[(i+1)*18+j+1 -1]                                + q_pad[(i+1)*18+j+1+1] +
        q_pad[(i+1)*18+j+1 -1-18]   + q_pad[(i+1)*18+j+1 -18] + q_pad[(i+1)*18+j+1 +1-18]) & 3'h7 | q[i*16+j]) == 3'h3;
    end

    if (load)
      q <= data;

  end

endmodule
```

#### model output
```verilog

```
---

### Prob145_circuit8
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clock
 - input  a
 - output p
 - output q

The module should implement a sequential circuit. Read the simulation
waveforms to determine what the circuit does, then implement it.

  time   clock   a   p   q
  0ns    0       0   x   x
  5ns    0       0   x   x
  10ns   0       0   x   x
  15ns   0       0   x   x
  20ns   0       0   x   x
  25ns   1       0   0   x
  30ns   1       0   0   x
  35ns   1       0   0   x
  40ns   1       0   0   x
  45ns   1       0   0   x
  50ns   1       0   0   x
  55ns   0       0   0   0
  60ns   0       0   0   0
  65ns   0       0   0   0
  70ns   0       1   0   0
  75ns   0       0   0   0
  80ns   0       1   0   0
  85ns   1       0   0   0
  90ns   1       1   1   0
  95ns   1       0   0   0
  100ns  1       1   1   0
  105ns  1       0   0   0
  110ns  1       1   1   0
  115ns  0       0   1   1
  120ns  0       1   1   1
  125ns  0       0   1   1
  130ns  0       1   1   1
  135ns  0       0   1   1
  140ns  0       0   1   1
  145ns  1       0   0   1
  150ns  1       0   0   1
  155ns  1       0   0   1
  160ns  1       0   0   1
  165ns  1       1   1   1
  170ns  1       0   0   1
  175ns  0       1   0   0
  180ns  0       0   0   0
  185ns  0       1   0   0
  190ns  0       0   0   0
```
</details>

#### reference
```verilog
module RefModule (
  input clock,
  input a,
  output reg p,
  output reg q
);

  always @(negedge clock)
    q <= a;

  always @(*)
    if (clock)
      p = a;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob146_fsm_serialdata
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  in
 - input  reset
 - output out_byte (8 bits)
 - output done

In many (older) serial communications protocols, each data byte is sent
along with a start bit and a stop bit, to help the receiver delimit bytes
from the stream of bits. One common scheme is to use one start bit (0), 8
data bits, and 1 stop bit (1). The line is also at logic 1 when nothing
is being transmitted (idle). Design a finite state machine that will
identify when bytes have been correctly received when given a stream of
bits. It needs to identify the start bit, wait for all 8 data bits, then
verify that the stop bit was correct. The module will also output the
correctly-received data byte. `out_byte` needs to be valid when `done` is
1, and is don't-care otherwise. If the stop bit does not appear when
expected, the FSM must wait until it finds a stop bit before attempting
to receive the next byte. Include a active-high synchronous reset. Note
that the serial protocol sends the least significant bit first. It should
assert done each time it finds a stop bit. Assume all sequential logic is
triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input in,
  input reset,
  output [7:0] out_byte,
  output done
);

  parameter B0=0, B1=1, B2=2, B3=3, B4=4, B5=5, B6=6, B7=7, START=8, STOP=9, DONE=10, ERR=11;
  reg [3:0] state;
  reg [3:0] next;

  reg [9:0] byte_r;

  always_comb begin
    case (state)
      START: next = in ? START : B0;  // start bit is 0
      B0: next = B1;
      B1: next = B2;
      B2: next = B3;
      B3: next = B4;
      B4: next = B5;
      B5: next = B6;
      B6: next = B7;
      B7: next = STOP;
      STOP: next = in ? DONE : ERR;  // stop bit is 1. Idle state is 1.
      DONE: next = in ? START : B0;
      ERR: next = in ? START : ERR;
    endcase
  end

  always @(posedge clk) begin
    if (reset) state <= START;
      else state <= next;
  end

  always @(posedge clk) begin
    byte_r <= {in, byte_r[9:1]};
  end

  assign done = (state==DONE);
  assign out_byte = done ? byte_r[8:1] : 8'hx;

endmodule
```

#### model output
```verilog

```
---

### Prob147_circuit10
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  a
 - input  b
 - output q
 - output state

The module should implement a sequential circuit. The circuit consists of
combinational logic and one bit of memory (i.e., one flip-flop). The
output of the flip-flop has been made observable through the output
state. Assume all sequential logic is triggered on the positive edge of
the clock.

Read the simulation waveforms to determine what the circuit does, then
implement it.

  time   clk a   b   state q
  0ns    0   1   x   x     x
  5ns    1   1   x   x     x
  10ns   0   0   0   x     x
  15ns   1   0   0   0     0
  20ns   0   0   0   0     0
  25ns   1   0   0   0     0
  30ns   0   0   0   0     0
  35ns   1   0   0   0     0
  40ns   0   0   0   0     0
  45ns   1   0   1   0     1
  50ns   0   0   1   0     1
  55ns   1   1   0   0     1
  60ns   0   1   0   0     1
  65ns   1   1   1   0     0
  70ns   0   1   1   0     0
  75ns   1   0   0   1     1
  80ns   0   0   0   1     1
  85ns   1   1   1   0     0
  90ns   0   1   1   0     0
  95ns   1   1   1   1     1
  100ns  0   1   1   1     1
  105ns  1   1   1   1     1
  110ns  0   1   1   1     1
  115ns  1   1   0   1     0
  120ns  0   1   0   1     0
  125ns  1   0   1   1     0
  130ns  0   0   1   1     0
  135ns  1   0   0   1     1
  140ns  0   0   0   1     1
  145ns  1   0   0   0     0
  150ns  0   0   0   0     0
  155ns  1   0   0   0     0
  160ns  0   0   0   0     0
  165ns  1   0   0   0     0
  170ns  0   0   0   0     0
  175ns  1   0   0   0     0
  180ns  0   0   0   0     0
  185ns  1   0   0   0     0
  190ns  0   0   0   0     0
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input a,
  input b,
  output q,
  output state
);

  reg c;
  always @(posedge clk)
    c <= a&b | a&c | b&c;

  assign q = a^b^c;
  assign state = c;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob148_2013_q2afsm
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  resetn
 - input  r (3 bits)
 - output g (3 bits)

The module should implement the FSM described by the state diagram shown
below:

  A        --r0=0,r1=0,r2=0--> A
  A        -------r0=1-------> B
  A        -----r0=0,r1=1----> C
  A        --r0=0,r1=0,r2=0--> D
  B (g0=1) -------r0=1-------> B
  B (g0=1) -------r0=0-------> A
  C (g1=1) -------r1=1-------> C
  C (g1=1) -------r1=0-------> A

Resetn is an active-low synchronous reset that resets into state A. This
FSM acts as an arbiter circuit, which controls access to some type of
resource by three requesting devices. Each device makes its request for
the resource by setting a signal _r[i]_ = 1, where _r[i]_ is either
_r[0]_, _r[1]_, or _r[2]_. Each r[i] is an input signal to the FSM, and
represents one of the three devices. The FSM stays in state _A_ as long
as there are no requests. When one or more request occurs, then the FSM
decides which device receives a grant to use the resource and changes to
a state that sets that device's _g[i]_ signal to 1. Each _g[i]_ is an
output from the FSM. There is a priority system, in that device 0 has a
higher priority than device 1, and device 2 has the lowest priority.
Hence, for example, device 2 will only receive a grant if it is the only
device making a request when the FSM is in state _A_. Once a device, _i_,
is given a grant by the FSM, that device continues to receive the grant
as long as its request, _r[i]_ = 1.

Implement a module that represents this FSM. Use separate always blocks
for the state table and the state flip-flops, as done in lectures.
Describe the FSM outputs, _g[i]_, using either continuous assignment
statement(s) or an always block (at your discretion). Assign any state
codes that you wish to use. Assume all sequential logic is triggered on
the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input resetn,
  input  [2:0] r,
  output [2:0] g
);

  parameter A=0, B=1, C=2, D=3;
  reg [1:0] state, next;

  always @(posedge clk) begin
    if (~resetn) state <= A;
    else state <= next;
  end

  always@(state,r) begin
    case (state)
      A: if (r[0]) next = B;
         else if (r[1]) next = C;
         else if (r[2]) next = D;
         else next = A;
      B: next = r[0] ? B : A;
      C: next = r[1] ? C : A;
      D: next = r[2] ? D : A;
      default: next = 'x;
    endcase
  end

  assign g[0] = (state == B);
  assign g[1] = (state == C);
  assign g[2] = (state == D);

endmodule
```

#### model output
```verilog

```
---

### Prob149_ece241_2013_q4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  s (3 bits)
 - output fr2
 - output fr1
 - output fr0
 - output dfr

A large reservior of water serves several users. In order to keep the
level of water succificently high, three sensors are placed vertically at
5-inch intervals. When the water level is above the highest sensor s[2],
the input flow rate should be zero. When the level is below the lowest
sensor s[0], the flow rate should be at maximum (both Nominal flow valve
and Supplemental flow valve opened). The flow rate when the level is
between the upper and lower sensors is determined by two factors: the
water level and the level previous to the last sensor change. Each water
level has a nominal flow rate associated with it as show in the table
below. If the sensor change indicates that the previous level was lower
than the current level, the flow rate should be increased by opening the
Supplemental flow valve (controlled by dfr).

  Water Level           | Sensors Asserted | Nominal Flow Rate Inputs to be Asserted
  Above s[2]            | s[0], s[1], s[2] | None
  Between s[2] and s[1] | s[0], s[1]       | fr0
  Between s[1] and s[0] | s[0]             | fr0, fr1
  Below s[0]            | None             | fr0, fr1, fr2

Also include an active-high synchronous reset that resets the state
machine to a state equivalent to if the water level had been low for a
long time (no sensors asserted, and all four outputs asserted).
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input [2:0] s,
  output reg fr2,
  output reg fr1,
  output reg fr0,
  output reg dfr
);

  parameter A2=0, B1=1, B2=2, C1=3, C2=4, D1=5;
  reg [2:0] state, next;

  always @(posedge clk) begin
    if (reset) state <= A2;
    else state <= next;
  end

  always@(*) begin
    case (state)
      A2: next = s[0] ? B1 : A2;
      B1: next = s[1] ? C1 : (s[0] ? B1 : A2);
      B2: next = s[1] ? C1 : (s[0] ? B2 : A2);
      C1: next = s[2] ? D1 : (s[1] ? C1 : B2);
      C2: next = s[2] ? D1 : (s[1] ? C2 : B2);
      D1: next = s[2] ? D1 : C2;
      default: next = 'x;
    endcase
  end
  reg [3:0] fr;
  assign {fr2, fr1, fr0, dfr} = fr;
  always_comb begin
    case (state)
      A2: fr = 4'b1111;
      B1: fr = 4'b0110;
      B2: fr = 4'b0111;
      C1: fr = 4'b0010;
      C2: fr = 4'b0011;
      D1: fr = 4'b0000;
      default: fr = 'x;
    endcase
  end

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob150_review2015_fsmonehot
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  d
 - input  done_counting
 - input  ack
 - input  state (10 bits)
 - output B3_next
 - output S_next
 - output S1_next
 - output Count_next
 - output Wait_next
 - output done
 - output counting
 - output shift_ena

The module should implement the following Moore state machine with 3
input (d, done_counting, ack) and 3 outputs (shift_ena, counting, done).
Unless otherwise stated in the diagram below, assume outputs are 0 and
inputs are don't cares.

state   (output)      --input--> next state
-------------------------------------------
  S     ()            --d=0--> S
  S     ()            --d=1--> S1
  S1    ()            --d=0--> S
  S1    ()            --d=1--> S11
  S11   ()            --d=0--> S110
  S11   ()            --d=1--> S11
  S110  ()            --d=0--> S
  S110  ()            --d=1--> B0
  B0    (shift_ena=1) --(always go to next cycle)--> B1
  B1    (shift_ena=1) --(always go to next cycle)--> B2
  B2    (shift_ena=1) --(always go to next cycle)--> B3
  B3    (shift_ena=1) --(always go to next cycle)--> Count
  Count (counting=1)  --done_counting=0--> Count
  Count (counting=1)  --done_counting=1--> Wait
  Wait  (done=1)      --ack=0--> Wait
  Wait  (done=1)      --ack=1--> S

At reset, the state machine starts in state "S". Derive next-state logic
equations and output logic equations by inspection assuming the following
one-hot encoding is used: (S, S1, S11, S110, B0, B1, B2, B3, Count, Wait)
= (10'b0000000001, 10'b0000000010, 10'b0000000100, ... , 10'b1000000000)

Derive state transition and output logic equations by inspection assuming
a one-hot encoding. Implement only the state transition logic and output
logic (the combinational logic portion) for this state machine.

Write code that generates the following signals:

 - B3_next -- Assert when next-state is B3 state
 - S_next -- Assert when next-state is S state
 - S1_next -- Assert when next-state is S1 state
 - Count_next -- Assert when next-state is Count state
 - Wait_next -- Assert when next-state is Wait state
 - done -- output logic
 - counting -- output logic
 - shift_ena -- output logic
```
</details>

#### reference
```verilog
module RefModule (
  input d,
  input done_counting,
  input ack,
  input [9:0] state, // 10-bit one-hot current state
  output B3_next,
  output S_next,
  output S1_next,
  output Count_next,
  output Wait_next,
  output done,
  output counting,
  output shift_ena
);

  parameter S=0, S1=1, S11=2, S110=3, B0=4, B1=5, B2=6, B3=7, Count=8, Wait=9;

  assign B3_next = state[B2];
  assign S_next = state[S]&~d | state[S1]&~d | state[S110]&~d | state[Wait]&ack;
  assign S1_next = state[S]&d;
  assign Count_next = state[B3] | state[Count]&~done_counting;
  assign Wait_next = state[Count]&done_counting | state[Wait]&~ack;

  assign done = state[Wait];
  assign counting = state[Count];
  assign shift_ena = |state[B3:B0];

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob151_review2015_fsm
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  data
 - input  done_counting
 - input  ack
 - output shift_ena
 - output counting
 - output done

The module should implement a timer that:

  (1) is started when a particular pattern (1101) is detected,
  (2) shifts in 4 more bits to determine the duration to delay,
  (3) waits for the counters to finish counting, and
  (4) notifies the user and waits for the user to acknowledge the timer.

In this problem, implement just the finite-state machine that controls
the timer. The data path (counters and some comparators) are not included
here.

The serial data is available on the data input pin. When the pattern 1101
is received, the state machine must then assert output shift_ena for
exactly 4 clock cycles. After that, the state machine asserts its
counting output to indicate it is waiting for the counters, and waits
until input done_counting is high.At that point, the state machine must
assert done to notify the user the timer has timed out, and waits until
input ack is 1 before being reset to look for the next occurrence of the
start sequence (1101). The state machine should have a active high
synchronous reset, setting the state to where it begins searching for the
input sequence 1101. Assume all sequential logic is triggered on the
positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input reset,
  input data,
  output reg shift_ena,
  output reg counting,
  input done_counting,
  output reg done,
  input ack
);

  typedef enum logic[3:0] {
    S, S1, S11, S110, B0, B1, B2, B3, Count, Wait
  } States;

  States state, next;

  always_comb begin
    case (state)
      S: next = States'(data ? S1: S);
      S1: next = States'(data ? S11: S);
      S11: next = States'(data ? S11 : S110);
      S110: next = States'(data ? B0 : S);
      B0: next = B1;
      B1: next = B2;
      B2: next = B3;
      B3: next = Count;
      Count: next = States'(done_counting ? Wait : Count);
      Wait: next = States'(ack ? S : Wait);
      default: next = States'(4'bx);
    endcase
  end

  always @(posedge clk) begin
    if (reset) state <= S;
    else state <= next;
  end

  always_comb begin
    shift_ena = 0; counting = 0; done = 0;
    if (state == B0 || state == B1 || state == B2 || state == B3)
      shift_ena = 1;
    if (state == Count)
      counting = 1;
    if (state == Wait)
      done = 1;

    if (|state === 1'bx) begin
      {shift_ena, counting, done} = 'x;
    end

  end

endmodule
```

#### model output
```verilog

```
---

### Prob152_lemmings3
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  bump_left
 - input  bump_right
 - input  ground
 - input  dig
 - output walk_left
 - output walk_right
 - output aaah
 - output digging

The game Lemmings involves critters with fairly simple brains. So simple
that we are going to model it using a finite state machine. In the
Lemmings' 2D world, Lemmings can be in one of two states: walking left
(walk_left is 1) or walking right (walk_right is 1). It will switch
directions if it hits an obstacle. In particular, if a Lemming is bumped
on the left (by receiving a 1 on bump_left), it will walk right. If it's
bumped on the right (by receiving a 1 on bump_right), it will walk left.
If it's bumped on both sides at the same time, it will still switch
directions.

In addition to walking left and right and changing direction when bumped,
when ground=0, the Lemming will fall and say "aaah!". When the ground
reappears (ground=1), the Lemming will resume walking in the same
direction as before the fall. Being bumped while falling does not affect
the walking direction, and being bumped in the same cycle as ground
disappears (but not yet falling), or when the ground reappears while
still falling, also does not affect the walking direction.

In addition to walking and falling, Lemmings can sometimes be told to do
useful things, like dig (it starts digging when dig=1). A Lemming can dig
if it is currently walking on ground (ground=1 and not falling), and will
continue digging until it reaches the other side (ground=0). At that
point, since there is no ground, it will fall (aaah!), then continue
walking in its original direction once it hits ground again. As with
falling, being bumped while digging has no effect, and being told to dig
when falling or when there is no ground is ignored. (In other words, a
walking Lemming can fall, dig, or switch directions. If more than one of
these conditions are satisfied, fall has higher precedence than dig,
which has higher precedence than switching directions.)

Implement a Moore state machine that models this behaviour. areset is
positive edge triggered asynchronous reseting the Lemming machine to walk
left. Assume all sequential logic is triggered on the positive edge of
the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input bump_left,
  input bump_right,
  input ground,
  input dig,
  output walk_left,
  output walk_right,
  output aaah,
  output digging
);

  parameter WL=0, WR=1, FALLL=2, FALLR=3, DIGL=4, DIGR=5;
  reg [2:0] state;
  reg [2:0] next;

  always_comb begin
    case (state)
      WL: if (!ground) next = FALLL;
        else if (dig) next = DIGL;
        else if (bump_left) next = WR;
        else next = WL;
      WR:
        if (!ground) next = FALLR;
        else if (dig) next = DIGR;
        else if (bump_right) next = WL;
        else next = WR;
      FALLL: next = ground ? WL : FALLL;
      FALLR: next = ground ? WR : FALLR;
      DIGL: next = ground ? DIGL : FALLL;
      DIGR: next = ground ? DIGR : FALLR;
    endcase
  end

  always @(posedge clk, posedge areset) begin
    if (areset) state <= WL;
      else state <= next;
  end

  assign walk_left = (state==WL);
  assign walk_right = (state==WR);
  assign aaah = (state == FALLL) || (state == FALLR);
  assign digging = (state == DIGL) || (state == DIGR);

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob153_gshare
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset

 - input  predict_valid,
 - input  predict_pc (7 bits)
 - output predict_taken
 - output predict_history (7 bits)

 - input  train_valid
 - input  train_taken
 - input  train_mispredicted
 - input  train_history (7 bits)
 - input  train_pc (7 bits)

The module should implement a gshare branch predictor with 7-bit pc and
7-bit global history, hashed (using xor) into a 7-bit index. This index
accesses a 128-entry table of two-bit saturating counters. The branch
predictor should contain a 7-bit global branch history register. The
branch predictor has two sets of interfaces: One for doing predictions
and one for doing training. The prediction interface is used in the
processor's Fetch stage to ask the branch predictor for branch direction
predictions for the instructions being fetched. Once these branches
proceed down the pipeline and are executed, the true outcomes of the
branches become known. The branch predictor is then trained using the
actual branch direction outcomes.

When a branch prediction is requested (predict_valid = 1) for a given pc,
the branch predictor produces the predicted branch direction and state of
the branch history register used to make the prediction. The branch
history register is then updated (at the next positive clock edge) for
the predicted branch.

When training for a branch is requested (train_valid = 1), the branch
predictor is told the pc and branch history register value for the branch
that is being trained, as well as the actual branch outcome and whether
the branch was a misprediction (needing a pipeline flush). Update the
pattern history table (PHT) to train the branch predictor to predict this
branch more accurately next time. In addition, if the branch being
trained is mispredicted, also recover the branch history register to the
state immediately after the mispredicting branch completes execution.

If training for a misprediction and a prediction (for a different,
younger instruction) occurs in the same cycle, both operations will want
to modify the branch history register. When this happens, training takes
precedence, because the branch being predicted will be discarded anyway.
If training and prediction of the same PHT entry happen at the same time,
the prediction sees the PHT state before training because training only
modifies the PHT at the next positive clock edge. The following timing
diagram shows the timing when training and predicting PHT entry 0 at the
same time. The training request at cycle 4 changes the PHT entry state in
cycle 5, but the prediction request in cycle 4 outputs the PHT state at
cycle 4, without considering the effect of the training request in cycle
4. Reset is asynchronous active-high.

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,

  input predict_valid,
  input [6:0] predict_pc,
  output predict_taken,
  output [6:0] predict_history,

  input train_valid,
  input train_taken,
  input train_mispredicted,
  input [6:0] train_history,
  input [6:0] train_pc
);

  parameter n = 7;
  logic [1:0] pht [2**n-1:0];

  parameter [1:0] SNT = 0, LNT = 1, LT = 2, ST = 3;

  logic [n-1:0] predict_history_r;
  wire [n-1:0] predict_index = predict_history_r ^ predict_pc;
  wire [n-1:0] train_index = train_history ^ train_pc;

  always@(posedge clk, posedge areset)
    if (areset) begin
      for (integer i=0; i<2**n; i=i+1)
        pht[i] = LNT;
      predict_history_r = 0;
        end  else begin
      if (predict_valid)
        predict_history_r <= {predict_history_r, predict_taken};
      if(train_valid) begin
        if(pht[train_index] < 3 && train_taken)
          pht[train_index] <= pht[train_index] + 1;
        else if(pht[train_index] > 0 && !train_taken)
          pht[train_index] <= pht[train_index] - 1;
        if (train_mispredicted)
          predict_history_r <= {train_history, train_taken};
      end
    end

  assign predict_taken = predict_valid ? pht[predict_index][1] : 1'bx;
  assign predict_history = predict_valid ? predict_history_r : {n{1'bx}};

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob154_fsm_ps2data
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  in (8 bits)
 - output out_bytes (24 bits)
 - output done

The module should implement a finite state machine that will search for
message boundaries when given an input byte stream. The algorithm we'll
use is to discard bytes until we see one with in[3]=1. We then assume
that this is byte 1 of a message, and signal the receipt of a message
once all 3 bytes have been received (done). The FSM should signal done in
the cycle immediately after the third byte of each message was
successfully received.

Implement the datapath module that will output the 24-bit (3 byte)
message whenever a packet is received (out_bytes[23:16] is the first
byte, out_bytes[15:8] is the second byte, etc.). The reset signal is
active high synchronous. out_bytes needs to be valid whenever the done
signal is asserted. You may output anything at other times (i.e.,
don't-care). Assume all sequential logic is triggered on the positive
edge of the clock.

Here is an example waveform:

  time   clk rst in  done out_bytes
  0ns    0   1    0  x         x
  5ns    1   1    0  0         x
  10ns   0   1    0  0         x
  15ns   1   0   2c  0         x
  20ns   0   0   2c  0         x
  25ns   1   0   81  0         x
  30ns   0   0   81  0         x
  35ns   1   0    9  0         x
  40ns   0   0    9  0         x
  45ns   1   0   6b  1    2c8109
  50ns   0   0   6b  1    2c8109
  55ns   1   0    d  0         x
  60ns   0   0    d  0         x
  65ns   1   0   8d  0         x
  70ns   0   0   8d  0         x
  75ns   1   0   6d  1    6b0d8d
  80ns   0   0   6d  1    6b0d8d
  85ns   1   0   12  0         x
  90ns   0   0   12  0         x
  95ns   1   0    1  0         x
  100ns  0   0    1  0         x
  105ns  1   0    d  1    6d1201
  110ns  0   0    d  1    6d1201
  115ns  1   0   76  0         x
  120ns  0   0   76  0         x
  125ns  1   0   3d  0         x
  130ns  0   0   3d  0         x
  135ns  1   0   ed  1     d763d
  140ns  0   0   ed  1     d763d
  145ns  1   0   8c  0         x
  150ns  0   0   8c  0         x
  155ns  1   0   f9  0         x
  160ns  0   0   f9  0         x
  165ns  1   0   ce  1    ed8cf9
  170ns  0   0   ce  1    ed8cf9
  175ns  1   0   c5  0         x
  180ns  0   0   c5  0         x
  185ns  1   0   aa  0         x
  190ns  0   0   aa  0         x
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input [7:0] in,
  input reset,
  output [23:0] out_bytes,
  output done
);

  parameter BYTE1=0, BYTE2=1, BYTE3=2, DONE=3;
  reg [1:0] state;
  reg [1:0] next;

  wire in3 = in[3];

  always_comb begin
    case (state)
      BYTE1: next = in3 ? BYTE2 : BYTE1;
      BYTE2: next = BYTE3;
      BYTE3: next = DONE;
      DONE: next = in3 ? BYTE2 : BYTE1;
    endcase
  end

  always @(posedge clk) begin
    if (reset) state <= BYTE1;
      else state <= next;
  end

  assign done = (state==DONE);

  reg [23:0] out_bytes_r;
  always @(posedge clk)
    out_bytes_r <= {out_bytes_r[15:0], in};

  // Implementations may vary: Allow user to do anything while the output
  // doesn't have to be valid.

  assign out_bytes = done ? out_bytes_r : 'x;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob155_lemmings4
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  areset
 - input  bump_left
 - input  bump_right
 - input  ground
 - input  dig
 - output walk_left
 - output walk_right
 - output aaah
 - output digging

The game Lemmings involves critters with fairly simple brains. So simple
that we are going to model it using a finite state machine. In the
Lemmings' 2D world, Lemmings can be in one of two states: walking left
(walk_left is 1) or walking right (walk_right is 1). It will switch
directions if it hits an obstacle. In particular, if a Lemming is bumped
on the left (by receiving a 1 on bump_left), it will walk right. If it's
bumped on the right (by receiving a 1 on bump_right), it will walk left.
If it's bumped on both sides at the same time, it will still switch
directions.

In addition to walking left and right and changing direction when bumped,
when ground=0, the Lemming will fall and say ""aaah!"". When the ground
reappears (ground=1), the Lemming will resume walking in the same
direction as before the fall. Being bumped while falling does not affect
the walking direction, and being bumped in the same cycle as ground
disappears (but not yet falling), or when the ground reappears while
still falling, also does not affect the walking direction.

In addition to walking and falling, Lemmings can sometimes be told to do
useful things, like dig (it starts digging when dig=1). A Lemming can dig
if it is currently walking on ground (ground=1 and not falling), and will
continue digging until it reaches the other side (ground=0). At that
point, since there is no ground, it will fall (aaah!), then continue
walking in its original direction once it hits ground again. As with
falling, being bumped while digging has no effect, and being told to dig
when falling or when there is no ground is ignored. (In other words, a
walking Lemming can fall, dig, or switch directions. If more than one of
these conditions are satisfied, fall has higher precedence than dig,
which has higher precedence than switching directions.)

Although Lemmings can walk, fall, and dig, Lemmings aren't invulnerable.
If a Lemming falls for too long then hits the ground, it can splatter. In
particular, if a Lemming falls for more than 20 clock cycles then hits
the ground, it will splatter and cease walking, falling, or digging (all
4 outputs become 0), forever (Or until the FSM gets reset). There is no
upper limit on how far a Lemming can fall before hitting the ground.
Lemmings only splatter when hitting the ground; they do not splatter in
mid-air.

Implement a Moore state machine that models this behaviour. areset is
positive edge triggered asynchronous reseting the Lemming machine to walk
left.

Assume all sequential logic is triggered on the positive edge of the
clock.
```
</details>

#### reference
```verilog
module RefModule (
  input clk,
  input areset,
  input bump_left,
  input bump_right,
  input ground,
  input dig,
  output walk_left,
  output walk_right,
  output aaah,
  output digging
);

  parameter WL=0, WR=1, FALLL=2, FALLR=3, DIGL=4, DIGR=5, DEAD=6;
  reg [2:0] state;
  reg [2:0] next;

  reg [4:0] fall_counter;

  always_comb begin
    case (state)
      WL: if (!ground) next = FALLL;
        else if (dig) next = DIGL;
        else if (bump_left) next = WR;
        else next = WL;
      WR:
        if (!ground) next = FALLR;
        else if (dig) next = DIGR;
        else if (bump_right) next = WL;
        else next = WR;
      FALLL: next = ground ? (fall_counter >= 20 ? DEAD : WL) : FALLL;
      FALLR: next = ground ? (fall_counter >= 20 ? DEAD : WR) : FALLR;
      DIGL: next = ground ? DIGL : FALLL;
      DIGR: next = ground ? DIGR : FALLR;
      DEAD: next = DEAD;
    endcase
  end

  always @(posedge clk, posedge areset) begin
    if (areset) state <= WL;
      else state <= next;
  end

  always @(posedge clk) begin
    if (state == FALLL || state == FALLR) begin
      if (fall_counter < 20)
        fall_counter <= fall_counter + 1'b1;
    end
    else
      fall_counter <= 0;
  end

  assign walk_left = (state==WL);
  assign walk_right = (state==WR);
  assign aaah = (state == FALLL) || (state == FALLR);
  assign digging = (state == DIGL) || (state == DIGR);

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---

### Prob156_review2015_fancytimer
<details>
<summary>prompt</summary>

```text
I would like you to implement a module named TopModule with the following
interface. All input and output ports are one bit unless otherwise
specified.

 - input  clk
 - input  reset
 - input  data
 - output count (4 bits)
 - output counting
 - output done
 - input  ack

The module should implement a timer with one input that:

  (1) is started when a particular input pattern (1101) is detected,
  (2) shifts in 4 more bits to determine the duration to delay,
  (3) waits for the counters to finish counting, and
  (4) notifies the user and waits for the user to acknowledge the timer.

The serial data is available on the data input pin. When the pattern 1101
is received, the circuit must then shift in the next 4 bits,
most-significant-bit first. These 4 bits determine the duration of the
timer delay, referred to as delay[3:0]. After that, the state machine
asserts its counting output to indicate it is counting. Once the 1101 and
delay[3:0] have been read, the circuit no longer looks at the data input
until it resumes searching after everything else is done.

The state machine must count for exactly (delay[3:0] + 1) * 1000 clock
cycles. e.g., delay=0 means count 1000 cycles, and delay=5 means count
6000 cycles. Also output the current remaining time. This should be equal
to delay for 1000 cycles, then delay-1 for 1000 cycles, and so on until
it is 0 for 1000 cycles.

When the circuit isn't counting, the count[3:0] output is don't-care
(whatever value is convenient for you to implement). At that point, the
circuit must assert done to notify the user the timer has timed out, and
waits until input ack is 1 before being reset to look for the next
occurrence of the start sequence (1101).

The circuit should reset into a state where it begins searching for the
input sequence 1101. The reset signal is active high synchronous. Assume
all sequential logic is triggered on the positive edge of the clock.
```
</details>

#### reference
```verilog
module RefModule (
  input wire clk,
  input wire reset,
  input wire data,
  output wire [3:0] count,
  output reg counting,
  output reg done,
  input wire ack
);

  typedef enum logic[3:0] {
    S, S1, S11, S110, B0, B1, B2, B3, Count, Wait
  } States;

  States state, next;

  reg shift_ena;
  reg [9:0] fcount;
  reg [3:0] scount;
  wire done_counting = (scount == 0) && (fcount == 999);

  always_comb begin
    case (state)
      S: next = States'(data ? S1: S);
      S1: next = States'(data ? S11: S);
      S11: next = States'(data ? S11 : S110);
      S110: next = States'(data ? B0 : S);
      B0: next = B1;
      B1: next = B2;
      B2: next = B3;
      B3: next = Count;
      Count: next = States'(done_counting ? Wait : Count);
      Wait: next = States'(ack ? S : Wait);
      default: next = States'(4'bx);
    endcase
  end

  always @(posedge clk) begin
    if (reset) state <= S;
    else state <= next;
  end

  always_comb begin
    shift_ena = 0; counting = 0; done = 0;
    if (state == B0 || state == B1 || state == B2 || state == B3)
      shift_ena = 1;
    if (state == Count)
      counting = 1;
    if (state == Wait)
      done = 1;

    if (|state === 1'bx) begin
      {shift_ena, counting, done} = 'x;
    end
  end

  // Shift register
  always @(posedge clk) begin
    if (shift_ena)
      scount <= {scount[2:0], data};
    else if (counting && fcount == 999)
      scount <= scount - 1'b1;
  end

  // Fast counter
  always @(posedge clk)
    if (!counting)
      fcount <= 10'h0;
    else if (fcount == 999)
      fcount <= 10'h0;
    else
      fcount <= fcount + 1'b1;

  assign count = counting ? scount : 'x;

endmodule
```

#### model output
```verilog
// VERILOG-EVAL: response did not use <CODE></CODE> correctly
```
---
