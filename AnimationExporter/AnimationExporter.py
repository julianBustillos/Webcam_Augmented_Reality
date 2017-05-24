import pymel.core as pm


def openFile(path, meshName):
    meshFile = open(path + "\\" + meshName, "w")
    return meshFile

def closeFile(meshFile):
    meshFile.close()
    return

def exportVertices(meshFile, node):
    for idx in range(len(node.vtx)):
        vertex = node.vtx[idx].getPosition()
        meshFile.write("v "+str(vertex[0])+" "+str(vertex[1])+" "+str(vertex[2])+"\n")
    meshFile.write("\n")
    return

def exportNormals(meshFile, node):
    for idx in range(len(node.vtx)):
        normal = node.vtx[idx].getNormal()
        meshFile.write("vn "+str(normal[0])+" "+str(normal[1])+" "+str(normal[2])+"\n")
    meshFile.write("\n")
    return

def exportColors(meshFile, node):
    for idx in range(len(node.vtx)):
        color = node.vtx[idx].getColor()
        meshFile.write("vc "+str(color[0])+" "+str(color[1])+" "+str(color[2])+" "+str(color[3])+"\n")
    meshFile.write("\n")
    return

def exportFaces(meshFile, node):
    for idx in range(len(node.faces)):
        indices = node.faces[idx].getVertices()
        meshFile.write("f "+str(indices[0])+" "+str(indices[1])+" "+str(indices[2])+"\n")
        if (len(indices) == 4):
            meshFile.write("f "+str(indices[2])+" "+str(indices[3])+" "+str(indices[0])+"\n")
    meshFile.write("\n")
    return

def exportKeyframes(meshFile, node):
    kfcount = pm.keyframe(node, query=True, kc=True)
    if (kfcount == 0):
        kflist = [1]
    else:
        kflist = pm.keyframe(node, query=True, index=(0,kfcount - 1))
    for kf in kflist:
        meshFile.write("kf "+str((kf - 1) / 24)+"\n")

        pm.currentTime(kf)
        scale = node.getScale()
        rotation = node.getRotation()
        translation = node.getTranslation()
        meshFile.write("s "+str(scale[0])+" "+str(scale[1])+" "+str(scale[2])+"\n")
        meshFile.write("r "+str(rotation[0])+" "+str(rotation[1])+" "+str(rotation[2])+"\n")
        meshFile.write("t "+str(translation[0])+" "+str(translation[1])+" "+str(translation[2])+"\n")

        meshFile.write("\n");
    return

def exportAnimation(path):

    for item in pm.ls(transforms=True):
        if (item != "front" and item != "persp" and item != "side" and item != "top"):
            meshName = item + ".kfobj"
            print "Exporting {} ...".format(meshName)
            node = pm.PyNode(item)
            meshFile = openFile(path, meshName)
            exportVertices(meshFile, node)
            exportNormals(meshFile, node)
            exportColors(meshFile, node)
            exportFaces(meshFile, node)
            exportKeyframes(meshFile, node)
            closeFile(meshFile)

    print "Exportation successful !"
    return


exportAnimation("C:\Users\Julian Bustillos\Downloads")
