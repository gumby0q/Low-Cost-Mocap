import { max } from "mathjs";
// import { MutableRefObject, useEffect, useRef } from "react";
import { Color, InstancedMesh, SphereGeometry, MeshLambertMaterial, Object3D } from "three";


export default function TreePoints({objectPoints, objectPointErrors }: {objectPoints: number[][][], objectPointErrors: number[][], }) {
// export default function TreePoints() {
  // Mock data
  // const objectPoints = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]; // Example data
  // const objectPointErrors = [0.1, 0.5, 0.9]; // Example error values

  // console.log("objectPoints", objectPoints)
  // console.log("objectPointErrors", objectPointErrors)
  const geometry = new SphereGeometry(0.008, 4, 4);
  const material = new MeshLambertMaterial();
  const instancedMesh = new InstancedMesh(geometry, material, objectPoints.length);

  // Helper functions
  const maxError = objectPointErrors.length !== 0 ? max(objectPointErrors) : 1;

  const errorToColour = (error: number) => {
    const scaledError = error / maxError;
    const logError = scaledError / (0.1 + scaledError);
    const tempColour = new Color(0x009999 + Math.round(logError * 0xff) * 0x10000);
    return tempColour;
  };

  const tempObject = new Object3D();
  objectPoints.forEach(([x, y, z], i) => {
    tempObject.position.set(x, z, y); // swapping y and z to match Three.js coordinate system
    tempObject.updateMatrix();
    instancedMesh.setMatrixAt(i, tempObject.matrix);
    instancedMesh.setColorAt(i, errorToColour(objectPointErrors[i]));
  });

  instancedMesh.instanceMatrix.needsUpdate = true;

  // scene.add(instancedMesh);

  return instancedMesh;
}

