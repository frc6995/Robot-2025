import json

reef = ["B", "C", "D", "E", "F", "G"]
intake = "SR3"

original = ""
with open("./STH-H.traj") as traj:
    original= traj.read()
print(original)
for pose in ["E", "F", "G"]:
    with open(f"./gen/ST{pose}-{pose}.traj", "w") as newTraj:
        newTraj.write(original.replace("H", pose))