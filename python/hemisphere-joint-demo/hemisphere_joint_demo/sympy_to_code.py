import dataclasses as dc
import os.path

import sympy as sp
import typing as ty

from collections import OrderedDict


@dc.dataclass
class Line:
    lhs: str
    rhs: str

    def make_decl(self) -> str:
        return f"double {self.lhs} = 0;"

    def make_impl(self):
        return f"{self.lhs} = {self.rhs};"

    @classmethod
    def lhs_from_expr(cls, expr: sp.Basic) -> str:
        lhs = ("_" + str(expr)
               .replace(" ", "")
               .replace("**", "_pow_")
               .replace("+", "_add_")
               .replace("-", "_add_")
               .replace("*", "_mul_")
               .replace("/", "_div_")
               .replace("(", "__lpar_")
               .replace(")", "_rpar__")
               )
        return lhs

    @classmethod
    def rhs_from_expr(cls, expr: sp.Basic, ctx: "Context") -> str:
        # Recurse.
        code_args = [expr_to_cpp(arg, ctx) for arg in expr.args]

        def par(code: str) -> str:
            return f"({code})"

        def op(op: str) -> str:
            return par(f" {op} ".join(code_args))

        def fn(func: str) -> str:
            return f"{func}({', '.join(code_args)})"

        if isinstance(expr, sp.Add):
            return op("+")

        elif isinstance(expr, sp.Subs):
            return op("-")

        elif isinstance(expr, sp.Mul):
            # Special case: a/b = a * (b^-1)
            return op("*")

        elif isinstance(expr, sp.Pow):
            assert len(code_args) == 2
            base, exponent = code_args
            if exponent == "-1":
                return par(f"1 / {base}")
            elif exponent == "2":
                return par(f"{base} * {base}")
            else:
                return fn("std::pow")

        elif isinstance(expr, sp.sin):
            return fn("std::sin")

        elif isinstance(expr, sp.cos):
            return fn("std::cos")

        else:
            raise TypeError(f"{expr.__class__}, {expr.func}({expr.args}) = {sp.srepr(expr)}")


    @classmethod
    def from_expr(cls, expr: sp.Basic, ctx: "Context"):
        lhs = Line.lhs_from_expr(expr)
        rhs = Line.rhs_from_expr(expr, ctx=ctx)
        return cls(lhs=lhs, rhs=rhs)


@dc.dataclass
class Context:

    name = "Expressions"

    function_args: ty.List[sp.Symbol] = dc.field(default_factory=list)
    named_expressions: ty.OrderedDict[sp.Basic, Line] = dc.field(default_factory=OrderedDict)
    function_results: ty.Dict[str, sp.Basic] = dc.field(default_factory=list)

    depth = 0
    indent = " " * 4

    def make_compute_args(self):
        return ", ".join(f"double {arg}" for arg in self.function_args)

    def make_compute_signature_decl(self):
        return f"void compute({self.make_compute_args()});"

    def make_compute_signature_impl(self):
        return f"void {self.name}::compute({self.make_compute_args()})"

    def make_decl_lines(self) -> ty.List[str]:
        lines = self._line_sum(
            [
                f"class {self.name}",
                "{",
                "public:",
                self.indent + "",
                self.indent + self.make_compute_signature_decl(),
                self.indent + "",
                self.indent + "// Input arguments:"
            ],
            [self.indent + f"double {arg} = 0;" for arg in self.function_args],
            [
                self.indent + "",
                self.indent + "// Results:"
            ],
            [self.indent + f"double {res} = 0;" for res in self.function_results],
            [
                self.indent + "",
                self.indent + "// Intermediate expressions:"
            ],
            [self.indent + line.make_decl() for expr, line in self.named_expressions.items()],
            [
                self.indent + "",
                "};",
            ]
        )
        return lines


    def make_impl_lines(self) -> ty.List[str]:
        lines = self._line_sum(
            [
                self.make_compute_signature_impl(),
                "{"
            ],
            [self.indent + f"this->{arg} = {arg};" for arg in self.function_args],
            [
                self.indent + "",
            ],
            [self.indent + line.make_impl() for expr, line in self.named_expressions.items()],
            [
                self.indent + "",
            ],
            [self.indent + Line(lhs=res, rhs=Line.lhs_from_expr(expr)).make_impl()
             for res, expr in self.function_results.items()],
            [
                "}",
            ],
        )
        return lines

    def make_header_lines(self):
        lines = self._line_sum(
            ["#pragma once"],
            [""] * 2,
            self.make_decl_lines(),
            [""] * 1,
        )
        return lines

    def make_source_lines(self):
        lines = self._line_sum(
            [f'#include "{self.name}.h"'],
            [""] * 1,
            ["#include <cmath>"],
            [""] * 2,
            self.make_impl_lines(),
            [""] * 1,
        )
        return lines

    @classmethod
    def format_lines(cls, lines: ty.List[str], line_numbers=False) -> str:
        if line_numbers:
            lines = [f"{i:>3} | {line}" for i, line in enumerate(lines)]
        return "\n".join(lines)

    @classmethod
    def write_lines(cls, lines: ty.List[str], filepath: str):
        with open(filepath, "w") as file:
            file.writelines([l.rstrip() + "\n" for l in lines])

    def _line_sum(self, *args):
        return sum(args, [])

    def build(self):
        for name, expr in self.function_results.items():
            expr_to_cpp(expr, self)


def expr_to_cpp(
        expr: sp.Basic,
        ctx: Context,
) -> str:
    indent = "  " * ctx.depth

    if len(expr.args) == 0:
        # Leaf.
        print(f"{indent}Leaf: {expr}")

        if isinstance(expr, sp.Symbol):
            # Must be part of local variables.
            assert expr in ctx.function_args

        elif isinstance(expr, sp.Number):
            # Will be turned into a literal.
            pass

        else:
            raise TypeError(f"Got expr {expr} of type {type(expr)}")

        return str(expr)

    else:
        # Non-leaf
        print(f"{indent}Node: {expr}")
        ctx.depth += 1

        line = Line.from_expr(expr, ctx=ctx)
        ctx.named_expressions[expr] = line

        ctx.depth -= 1

        return line.lhs



if __name__ == '__main__':
    from sympy import sin, cos, sqrt

    # Actuation
    a1, a2 = sp.symbols('a1 a2')
    # Constants defining deometry
    lever, theta0 = sp.symbols('lever theta0')
    # P1_z=f(motor1)
    # P1_z=f(motor2)

    # for 90°
    ex = 2 * lever * (lever ** 5 * sin(theta0) - lever ** 3 * a1 * a2 * sin(theta0) - lever ** 3 * a2 ** 2 * sin(theta0) + lever * a1 * a2 ** 3 * sin(theta0) - a2 * sqrt(-2 * lever ** 8 * sin(theta0) ** 2 + lever ** 8 + 2 * lever ** 6 * a1 ** 2 * sin(theta0) ** 2 - lever ** 6 * a1 ** 2 + 2 * lever ** 6 * a1 * a2 * sin(theta0) ** 2 + 2 * lever ** 6 * a2 ** 2 * sin(theta0) ** 2 - lever ** 6 * a2 ** 2 - 2 * lever ** 4 * a1 ** 3 * a2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 ** 2 * a2 ** 2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 4 * a2 ** 2 + 2 * lever ** 2 * a1 ** 3 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(theta0) / (sqrt(lever ** 2 - a2 ** 2) * (lever ** 4 - a1 ** 2 * a2 ** 2))
    ey = 2 * lever * (lever ** 5 * sin(theta0) - lever ** 3 * a1 ** 2 * sin(theta0) - lever ** 3 * a1 * a2 * sin(theta0) + lever * a1 ** 3 * a2 * sin(theta0) - a1 * sqrt(-2 * lever ** 8 * sin(theta0) ** 2 + lever ** 8 + 2 * lever ** 6 * a1 ** 2 * sin(theta0) ** 2 - lever ** 6 * a1 ** 2 + 2 * lever ** 6 * a1 * a2 * sin(theta0) ** 2 + 2 * lever ** 6 * a2 ** 2 * sin(theta0) ** 2 - lever ** 6 * a2 ** 2 - 2 * lever ** 4 * a1 ** 3 * a2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 ** 2 * a2 ** 2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 4 * a2 ** 2 + 2 * lever ** 2 * a1 ** 3 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(theta0) / (sqrt(lever ** 2 - a1 ** 2) * (lever ** 4 - a1 ** 2 * a2 ** 2))
    ez = 2 * lever * (lever * (lever ** 4 - a1 ** 2 * a2 ** 2) * (a1 + a2) * sin(theta0) + (lever ** 2 + a1 * a2) * sqrt(lever ** 2 * (lever ** 2 * a1 + lever ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(theta0) ** 2 + (-lever ** 4 + a1 ** 2 * a2 ** 2) * (-2 * lever ** 4 * cos(theta0) ** 2 + lever ** 4 + lever ** 2 * a1 ** 2 * cos(theta0) ** 2 + lever ** 2 * a2 ** 2 * cos(theta0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(theta0) / ((lever ** 2 + a1 * a2) * (lever ** 4 - a1 ** 2 * a2 ** 2))

    ctx = Context(
        function_args=[a1, a2, lever, theta0],
        function_results={
            "ex": ex,
            "ey": ey,
            "ez": ez,
        }
    )
    ctx.build()

    output_dir = "/home/rkartmann/code/simox/VirtualRobot/examples/HemisphereJoint/"
    header_path = os.path.join(output_dir, ctx.name + ".h")
    source_path = os.path.join(output_dir, ctx.name + ".cpp")

    header_lines = ctx.make_header_lines()
    source_lines = ctx.make_source_lines()

    print("Declaration:")
    print(ctx.format_lines(header_lines, line_numbers=True))
    print("Implementation:")
    print(ctx.format_lines(source_lines, line_numbers=True))

    print("Writing files...")
    print(f"- {header_path}")
    print(ctx.write_lines(header_lines, header_path))
    print(f"- {source_path}")
    print(ctx.write_lines(source_lines, source_path))

    print("Done.")
