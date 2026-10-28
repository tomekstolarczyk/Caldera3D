"""Download the demo scene and convert it to the ASCII .ply format read by Caldera3D.

The scene is table_scene_lms400 from the PCL data repository. It is stored as a
`binary_compressed` .pcd: an LZF-compressed block with the fields laid out one after
another (all x, then all y, ...). The converter below needs only NumPy.

Usage: python3 scripts/download_data.py [--out data/table_scene_lms400.ply]
"""

import argparse
import pathlib
import struct
import urllib.request

import numpy as np

URL = "https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd"

PCD_TYPES = {("F", 4): np.float32, ("F", 8): np.float64,
             ("U", 1): np.uint8, ("U", 2): np.uint16, ("U", 4): np.uint32,
             ("I", 1): np.int8, ("I", 2): np.int16, ("I", 4): np.int32}


def lzf_decompress(src: bytes, out_len: int) -> bytes:
    """LZF decoder (the format used by PCL's binary_compressed .pcd files)."""
    out = bytearray(out_len)
    ip = op = 0
    while ip < len(src):
        ctrl = src[ip]
        ip += 1
        if ctrl < 32:  # literal run of ctrl + 1 bytes
            n = ctrl + 1
            out[op:op + n] = src[ip:ip + n]
            ip += n
            op += n
        else:  # back-reference: copy `length` bytes from `ref`
            length = ctrl >> 5
            ref = op - ((ctrl & 0x1F) << 8) - 1
            if length == 7:
                length += src[ip]
                ip += 1
            ref -= src[ip]
            ip += 1
            length += 2
            if ref + length <= op:  # no overlap - slice copy
                out[op:op + length] = out[ref:ref + length]
                op += length
            else:  # overlapping copy (run-length style) must go byte by byte
                for _ in range(length):
                    out[op] = out[ref]
                    op += 1
                    ref += 1
    if op != out_len:
        raise ValueError(f"LZF: expected {out_len} bytes, got {op}")
    return bytes(out)


def read_pcd_xyz(raw: bytes) -> np.ndarray:
    """Parse a .pcd file (ascii / binary / binary_compressed) and return an (N, 3) float32 array."""
    header = {}
    pos = 0
    while True:
        end = raw.index(b"\n", pos)
        line = raw[pos:end].decode("ascii").strip()
        pos = end + 1
        if not line or line.startswith("#"):
            continue
        key, *values = line.split()
        header[key] = values
        if key == "DATA":
            break

    fields = header["FIELDS"]
    sizes = [int(v) for v in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    n = int(header["POINTS"][0])
    dtypes = [PCD_TYPES[(t, s)] for t, s in zip(types, sizes)]
    data_kind = header["DATA"][0]

    columns = {}
    if data_kind == "ascii":
        table = np.loadtxt(raw[pos:].decode("ascii").splitlines(), ndmin=2)
        col = 0
        for f, c in zip(fields, counts):
            columns[f] = table[:, col]
            col += c
    elif data_kind == "binary":  # array of structs
        dt = np.dtype([(f, d, (c,)) if c > 1 else (f, d) for f, d, c in zip(fields, dtypes, counts)])
        arr = np.frombuffer(raw, dtype=dt, count=n, offset=pos)
        columns = {f: arr[f] for f in fields}
    elif data_kind == "binary_compressed":  # struct of arrays, LZF-compressed
        comp_size, uncomp_size = struct.unpack_from("<II", raw, pos)
        data = lzf_decompress(raw[pos + 8:pos + 8 + comp_size], uncomp_size)
        offset = 0
        for f, d, s, c in zip(fields, dtypes, sizes, counts):
            columns[f] = np.frombuffer(data, dtype=d, count=n * c, offset=offset)
            offset += n * s * c
    else:
        raise ValueError(f"unsupported PCD DATA type: {data_kind}")

    xyz = np.stack([columns["x"], columns["y"], columns["z"]], axis=1).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def write_ascii_ply(path: pathlib.Path, xyz: np.ndarray) -> None:
    header = ("ply\nformat ascii 1.0\n"
              f"element vertex {len(xyz)}\n"
              "property float x\nproperty float y\nproperty float z\nend_header")
    np.savetxt(path, xyz, fmt="%.9g", header=header, comments="")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="data/table_scene_lms400.ply", type=pathlib.Path)
    args = parser.parse_args()

    print(f"Downloading {URL}")
    with urllib.request.urlopen(URL) as resp:
        raw = resp.read()

    xyz = read_pcd_xyz(raw)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_ascii_ply(args.out, xyz)
    print(f"Saved {len(xyz)} points to {args.out}")


if __name__ == "__main__":
    main()
