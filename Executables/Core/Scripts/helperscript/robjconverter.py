# obj_to_robj.py
import sys
import os

def parse_obj(filename):
    vertices = []
    colors = []
    faces = []

    vertex_map = {}  # optional: map to handle duplicate vertices
    index = 0

    with open(filename, 'r') as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line:
                continue
            parts = line.split()
            if parts[0] == 'v':
                x, y, z = map(float, parts[1:4])
                vertices.append((x, y, z))
                colors.append((1.0, 1.0, 1.0))
            elif parts[0] == 'f':
                for p in parts[1:]:
                    # OBJ may be v, v/vt, or v//vn
                    v_idx = int(p.split('/')[0]) - 1
                    faces.append(v_idx)
    return vertices, colors, faces


def write_robj(filename, vertices, colors, faces):
    with open(filename, 'w') as f:
        f.write(f"float vertexCoords[{len(vertices)}] = {{\n")
        for i, v in enumerate(vertices):
            f.write(f"{v}")
            if i != len(vertices)-1:
                f.write(", ")
            if (i+1) % 3 == 0:
                f.write("\n")
        f.write("};\n\n")

        f.write(f"float vertexColors[{len(colors)}] = {{\n")
        for i, c in enumerate(colors):
            f.write(f"{c}")
            if i != len(colors)-1:
                f.write(", ")
            if (i+1) % 3 == 0:
                f.write("\n")
        f.write("};\n\n")

        f.write(f"int elementArray[{len(faces)}] = {{\n")
        for i, idx in enumerate(faces):
            f.write(f"{idx}")
            if i != len(faces)-1:
                f.write(", ")
            if (i+1) % 4 == 0:  # group 4 per line (quad style)
                f.write("\n")
        f.write("};\n")

def main():
    if len(sys.argv) != 3:
        print("Usage: python obj_to_robj.py input.obj output.robj")
        return

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.isfile(input_file):
        print(f"File {input_file} does not exist!")
        return

    vertices, colors, faces = parse_obj(input_file)
    write_robj(output_file, vertices, colors, faces)
    print(f"Converted {input_file} → {output_file}")
    print(f"Vertices: {len(vertices)//3}, Colors: {len(colors)//3}, Faces: {len(faces)//3}")

if __name__ == "__main__":
    main()
# Usage: python obj_to_robj.py input.obj output.robj