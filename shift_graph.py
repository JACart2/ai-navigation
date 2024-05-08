filename = "./navigation/maps/main.gml"
write_file = "./navigation/maps/main_shift3.gml"
f = open(filename, "r")
new = open(write_file, "w")
dx = -0.5
# dy was originall -1.5 but i didnt wanna touch it for the third graph file
dy = -2.0
while True:
    line = f.readline()
    if line == "":
        break
    new.write(line)
    if line[:6] == "  node":
        new.write(f.readline())
        new.write(f.readline())
        new.write(f.readline())
        x = float(f.readline().strip()[4:]) + dx
        y = float(f.readline().strip()[4:]) + dy
        new.write(f"      pos {x}\n")
        new.write(f"      pos {y}\n")
        print(x)
        print(y)
        print()


f.close()
