#!/usr/bin/env python

import click
import os
import glob

import pymeshlab

from typing import List
from rich import print


def find_simox_executable(filename: str):
    try:
        simox_dir = os.environ["Simox_DIR"]
    except KeyError:
        return None
    else:
        executable = os.path.join(simox_dir, "bin", filename)
        return executable if os.path.isfile(executable) else None


def convert_wrl_to_obj(
        in_file: str,
        out_file: str,
):
    if in_file.endswith(".iv") and out_file.endswith(".wrl"):
        executable = find_simox_executable("Iv2Wrl") or "Iv2Wrl"
        command = f'{executable} --input "{in_file}" --output "{out_file}"'
        os.system(command)

    else:
        assert os.path.isfile(in_file), f"Input file '{in_file}' must exist (wdir: " + os.path.abspath(".") + ")."
        ms = pymeshlab.MeshSet()
        try:
            ms.load_new_mesh(in_file)
        except pymeshlab.pmeshlab.PyMeshLabException as e:
            print(f"Failed load input file '{in_file}':", e)
        else:
            ms.save_current_mesh(out_file)


def main(
        output_formats=("obj",),
        input_format="wrl",
        directory=".",
        overwrite=False,
        inplace=False,
):
    working_dir = os.path.abspath(".")

    in_files = sorted(glob.glob(os.path.join(directory, "**", f"*.{input_format}"), recursive=True))
    in_files = list(filter(lambda f: os.path.isfile(f), in_files))
    print(f"Found {len(in_files)} .{input_format} files.")

    for output_fmt in output_formats:
        for i, in_file in enumerate(in_files):
            os.chdir(working_dir)  # Something appears to sometimes change the working directory.

            prefix = f"[{i+1}]"
            out_file = os.path.splitext(in_file)[0] + f".{output_fmt}"
            if not inplace:
                out_file = os.path.join(output_fmt, out_file)
            os.makedirs(os.path.dirname(out_file), exist_ok=True)

            if overwrite or not os.path.exists(out_file):
                print(f"{prefix} Convert '{in_file}' to '{out_file}' ...")
                convert_wrl_to_obj(in_file, out_file)

            else:
                print(f"{prefix} Skip '{in_file}' ('{out_file}' exists).")
                continue


@click.command()
@click.argument("output-formats", nargs=-1, required=True)
@click.option("-i", "--input-format", default="wrl", help="Input file format.")
@click.option("-d", "--directory", default=".", help="Overwrite existing output files.")
@click.option("-f", "--force", default=False, is_flag=True,
              help="Overwrite existing output files.")
@click.option("-p", "--inplace", default=False, is_flag=True,
              help="Create output meshes next to input files. Otherwise, output meshes are created in a separate "
                   "directory matching the original directory structure.")
def cli(
        output_formats: List[str],
        input_format: str,
        directory: str,
        force: bool,
        inplace: bool,
):
    """
    Convert meshes of the input format to output formats.

    OUTPUT_FORMATS is a list of output mesh file extensions, e.g. 'obj dae',
    to which input mesh files should be converted.
    """
    main(
        output_formats=output_formats,
        input_format=input_format,
        overwrite=force,
        directory=directory,
        inplace=inplace,
    )


if __name__ == '__main__':
    cli()
