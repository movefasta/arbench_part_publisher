#!/usr/bin/env python
from arbench_part_publisher_node import PartsPublisher, ArbenchPart
import rospy
import os
import argparse
import csv
import math

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Node that spawns a set of parts using the arbench_part_pu"
        + "blisher. Subdirectories in the folder are assumed to be of the form"
        + ":\n ./PARTNAME/FILENAME.stl, ./PARTNAME/placements.csv\n where plac"
        + "ements"
    )
    parser.add_argument("-d",
                        help="Directory to look through",
                        type=str, required=True)
    parser.add_argument("-mf",
                        help="Mesh filename. Defaults to simple.stl",
                        type=str, default="simple.stl")
    parser.add_argument("-pf",
                        help="Placement filename. Defaults to placements.csv, "
                        + "used for creating multiple copies of a mesh.",
                        type=str, default="placements.csv")
    parser.add_argument("-pl",
                        help="Part list. List of part names indicating the "
                        + "ones to load.",
                        type=str, default="")
    args, unknown_args = parser.parse_known_args()
    mf = args.mf
    pf = args.pf
    meshdir = args.d
    if args.pl != "":
        with open(args.pl, "r") as partlistfile:
            pl = partlistfile.read().splitlines()
    else:
        pl = []
    # Find folders with the desired mesh filename in it in the directory
    items = os.listdir(meshdir)
    dirs = [i for i in items if os.path.isdir(os.path.join(meshdir, i))]
    dirs = [d for d in dirs if os.path.exists(os.path.join(meshdir, d, mf))]
    dirs = [os.path.join(meshdir, d) for d in dirs]
    # Create a list with descriptions of the different meshes
    partlist = []
    for d in dirs:
        # Check if the current directory
        if len(pl) > 0:
            if not os.path.split(d)[-1] in pl:
                continue
        # Initial placement of part
        placecsvpath = os.path.join(d, "placements.csv")
        if os.path.exists(placecsvpath):
            with open(placecsvpath, "rb") as csvfile:
                placerdr = csv.reader(csvfile, delimiter=",")
                placerdr.next()  # Skip header
                for row in placerdr:
                    lbl = row[0].replace("-", "_")
                    part = ArbenchPart(
                        label=lbl,
                        mesh_uri="file://"+os.path.join(d, mf)
                    )
                    x = float(row[1])
                    y = float(row[2])
                    z = float(row[3])
                    qx = math.sin(float(row[7])/2.)*float(row[4])
                    qy = math.sin(float(row[7])/2.)*float(row[5])
                    qz = math.sin(float(row[7])/2.)*float(row[6])
                    qw = math.cos(float(row[7])/2.)
                    tr = part.frames[lbl + "_part_frame"]
                    tr.transform.translation.x = x
                    tr.transform.translation.y = y
                    tr.transform.translation.z = z
                    tr.transform.rotation.x = qx
                    tr.transform.rotation.y = qy
                    tr.transform.rotation.z = qz
                    tr.transform.rotation.w = qw
                    partlist.append(part)
        else:
            part = ArbenchPart(label=os.path.split(d)[-1],
                               mesh_uri="file://"+os.path.join(d, mf))
            partlist.append(part)
        # Add features to the part
        featurescsvpath = os.path.join(d, "features.csv")
        if os.path.exists(featurescsvpath):
            with open(featurescsvpath, "rb") as csvfile:
                featurerdr = csv.reader(csvfile, delimiter=",")
                featurerdr.next()  # Skip header
                for row in featurerdr:
                    lbl = row[0].replace("-", "_")
                    x = float(row[1])
                    y = float(row[2])
                    z = float(row[3])
                    qx = math.sin(float(row[7])/2.)*float(row[4])
                    qy = math.sin(float(row[7])/2.)*float(row[5])
                    qz = math.sin(float(row[7])/2.)*float(row[6])
                    qw = math.cos(float(row[7])/2.)
                    part.add_feature(lbl, x, y, z, qx, qy, qz, qw)
    # Start up the publisher
    part_pub = PartsPublisher(partlist, rate=10, mesh_freq=10)
    try:
        part_pub.run()
    except rospy.ROSInterruptException:
        pass
