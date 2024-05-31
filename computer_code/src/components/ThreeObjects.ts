import { ConeGeometry, MeshPhongMaterial, Color, InstancedMesh, Matrix4, Object3D } from "three";
import { numberToHexColor } from "../shared/styles/scripts/helpers";

// export default function Objects({filteredObjectsRef, count}: {filteredObjectsRef: MutableRefObject<Object[]>, count: number}) {
//   let objects = filteredObjectsRef.current.flat()

//   const instancedMeshRef = useRef<InstancedMesh<BufferGeometry<NormalBufferAttributes>, Material | Material[]>>()
//   const temp = new Object3D()
//   const tempColour = new Color()

//   let arrowDefaultDirection = new Vector3
//   arrowDefaultDirection.set(1,0,0)

//   let arrowDefaultLocation = new Vector3
//   arrowDefaultLocation.set(0,0,0)

//   useEffect(() => {
//     objects.forEach(({pos, heading, droneIndex}, i) => {
//       temp.position.set(pos[0], pos[2], pos[1]) // y is up in threejs
//       let threeRotationMatrixY = new Matrix4
//       threeRotationMatrixY.makeRotationY(heading)
//       let threeRotationMatrixZ = new Matrix4
//       threeRotationMatrixZ.makeRotationZ(Math.PI/2)
//       threeRotationMatrixY.multiply(threeRotationMatrixZ)
//       temp.setRotationFromMatrix(threeRotationMatrixY)
//       temp.updateMatrix() 
//       instancedMeshRef.current!.setMatrixAt(i, temp.matrix)
//       instancedMeshRef.current!.setColorAt(i, tempColour.set(numberToHexColor(droneIndex, 2))) 
//     })
//     instancedMeshRef.current!.instanceMatrix.needsUpdate = true 
//   }, [count])
//   return (
//     <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, objects.length]}>
//         <coneGeometry args={[0.005, 0.02, 5, 5]}/>
//         <meshPhongMaterial/>
//     </instancedMesh>
//   )
// }


// export default function ThreeObjects({ filteredObjects }: { filteredObjects: Object[] }) {
export default function ThreeObjects() {
    // Mock data
    const objects = [
        { pos: [1, 2, 3], heading: Math.PI / 4, droneIndex: 0 },
        { pos: [4, 5, 6], heading: Math.PI / 2, droneIndex: 1 }
    ];

    const geometry = new ConeGeometry(0.005, 0.02, 5, 5);
    const material = new MeshPhongMaterial();
    const instancedMesh = new InstancedMesh(geometry, material, objects.length);

    const tempObject = new Object3D();
    const tempColour = new Color();

    // // Function to update colors based on an index
    // function numberToHexColor(index: number, digits: number) {
    //     return (index + 1) * (0xffffff / (Math.pow(10, digits)));
    // }

    // Update the instance mesh with positions and rotations
    const updateInstances = () => {
        objects.forEach(({ pos, heading, droneIndex }, i) => {
            tempObject.position.set(pos[0], pos[2], pos[1]); // y is up in Three.js
            const threeRotationMatrixY = new Matrix4().makeRotationY(heading);
            const threeRotationMatrixZ = new Matrix4().makeRotationZ(Math.PI / 2);
            threeRotationMatrixY.multiply(threeRotationMatrixZ);
            tempObject.setRotationFromMatrix(threeRotationMatrixY);
            tempObject.updateMatrix();
            instancedMesh.setMatrixAt(i, tempObject.matrix);
            instancedMesh.setColorAt(i, tempColour.set(numberToHexColor(droneIndex, 2)));
        });
        instancedMesh.instanceMatrix.needsUpdate = true;
    };

    updateInstances();

    // scene.add(instancedMesh);

    return instancedMesh;
}

