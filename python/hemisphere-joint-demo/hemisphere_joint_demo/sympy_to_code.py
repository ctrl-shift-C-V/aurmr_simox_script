import dataclasses as dc

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
        lhs = "_" + str(expr)
        long = False
        if long:
            lhs = (lhs.replace(" ", "")
                   .replace("**", "_pow_")
                   .replace("+", "_add_")
                   .replace("-", "_sub_")
                   .replace("*", "_mul_")
                   .replace("/", "_div_")
                   .replace(".", "_dot_")
                   .replace("(", "__lpar_")
                   .replace(")", "_rpar__")
                   )
        else:
            lhs = (lhs.replace(" ", "")
                   .replace("**", "_p_")
                   .replace("+", "_a_")
                   .replace("-", "_s_")
                   .replace("*", "_m_")
                   .replace("/", "_d_")
                   .replace(".", "_t_")
                   .replace("(", "_l_")
                   .replace(")", "_r_")
                   )
        return lhs

    @classmethod
    def rhs_from_expr(cls, expr: sp.Basic, cpp: "SympyToCpp") -> str:
        # Recurse.
        code_args = [expr_to_cpp(arg, cpp) for arg in expr.args]

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

        elif isinstance(expr, sp.asin):
            return fn("std::asin")

        elif isinstance(expr, sp.acos):
            return fn("std::acos")

        elif isinstance(expr, sp.exp):
            return fn("std::exp")

        else:
            raise TypeError(f"{expr.__class__}, {expr.func}({expr.args}) = {sp.srepr(expr)}")


    @classmethod
    def from_expr(cls, expr: sp.Basic, cpp: "SympyToCpp"):
        lhs = Line.lhs_from_expr(expr)
        rhs = Line.rhs_from_expr(expr, cpp=cpp)
        return cls(lhs=lhs, rhs=rhs)


@dc.dataclass
class SympyToCpp:

    name: str = "Expressions"
    namespace: str = "VirtualRobot::hemisphere"

    function_args: ty.List[sp.Symbol] = dc.field(default_factory=list)
    named_expressions: "ty.OrderedDict[sp.Basic, Line]" = dc.field(default_factory=OrderedDict)
    function_results: ty.Dict[str, sp.Basic] = dc.field(default_factory=list)

    depth = 0
    indent = " " * 4

    def build(self):
        for name, expr in self.function_results.items():
            expr_to_cpp(expr, self)

    def make_compute_args(self):
        return ", ".join(f"double {arg}" for arg in self.function_args)

    def make_compute_signature_decl(self):
        return f"void compute({self.make_compute_args()});"

    def make_compute_signature_impl(self):
        return f"void {self.name}::compute({self.make_compute_args()})"

    def make_namespace_begin_end(self):
        begin = [
            f"namespace {self.namespace}",
            "{",
            "",
        ]
        end = [
            "",
            "}"
        ]
        return begin, end

    def make_generation_note(self):
        import datetime
        now = datetime.datetime.now()
        now = now.strftime("%Y-%m-%d %H:%M")
        return [
            f"/*",
            f" * This file was generated automatically on {now}.",
            f" */"
            "",
            "",
        ]

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
        ns_begin, ns_end = self.make_namespace_begin_end()
        lines = self._line_sum(
            self.make_generation_note(),
            ["#pragma once"],
            [""] * 2,
            ns_begin,
            self.make_decl_lines(),
            ns_end,
            [""] * 1,
        )
        return lines

    def make_source_lines(self):
        ns_begin, ns_end = self.make_namespace_begin_end()
        lines = self._line_sum(
            self.make_generation_note(),
            [f'#include "{self.name}.h"'],
            [""] * 1,
            ["#include <cmath>"],
            [""] * 2,
            ns_begin,
            self.make_impl_lines(),
            ns_end,
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


def expr_to_cpp(
        expr: sp.Basic,
        cpp: SympyToCpp,
) -> str:
    indent = "  " * cpp.depth

    if len(expr.args) == 0:
        # Leaf.
        print(f"{indent}Leaf: {expr}")

        if isinstance(expr, sp.Symbol):
            # Must be part of local variables.
            assert expr in cpp.function_args
            return str(expr)

        elif isinstance(expr, sp.Number):
            # Will be turned into a literal.
            # The number can also be something like (1/2), where "1 / 2" would be 0 in C++.
            # So we pre-evaluate these constants in Python, and pass the result literal to C++.
            return str(eval(str(expr)))

        else:
            raise TypeError(f"Got expr {expr} of type {type(expr)}")

    else:
        # Non-leaf
        print(f"{indent}Node: {expr}")
        cpp.depth += 1

        line = Line.from_expr(expr, cpp=cpp)
        cpp.named_expressions[expr] = line

        cpp.depth -= 1

        return line.lhs
