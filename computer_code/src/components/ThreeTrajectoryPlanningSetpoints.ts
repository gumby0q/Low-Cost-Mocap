// import { max, multiply } from "mathjs";
// import { useEffect, useRef } from "react";
// import { Color, InstancedMesh, Matrix4, Object3D } from "three";
import { Color, InstancedMesh, Object3D, SphereGeometry, MeshLambertMaterial } from "three";

// export default function ThreeTrajectoryPlanningSetpoints({ trajectoryPlanningSetpoints, NUM_DRONES }: { trajectoryPlanningSetpoints: number[][], NUM_DRONES: number }) {
export default function ThreeTrajectoryPlanningSetpoints() {
    // const instancedMeshRef = useRef<InstancedMesh>()
    // const temp = new Object3D()
    // const tempColour = new Color()

    // useEffect(() => {
    //   for (let droneIndex = 0; droneIndex < NUM_DRONES; droneIndex++) {
    //     const positions = trajectoryPlanningSetpoints.map(x => x.slice(droneIndex * 3, (droneIndex + 1) * 3))
    //     positions.forEach((pos, i) => {
    //       // console.log("pos", pos) /* has epty objects :/ */
    //       const [x, y, z] = pos;
    //       temp.position.set(x, z, y) // y is up in threejs
    //       temp.updateMatrix()
    //       instancedMeshRef.current!.setMatrixAt(i + (droneIndex * trajectoryPlanningSetpoints.length), temp.matrix)
    //       instancedMeshRef.current!.setColorAt(i + (droneIndex * trajectoryPlanningSetpoints.length), tempColour.set(0x00ffff))
    //     })
    //   }

    //   instancedMeshRef.current!.instanceMatrix.needsUpdate = true
    // }, [trajectoryPlanningSetpoints])

    // return (
    //   <instancedMesh ref={instancedMeshRef} args={[undefined, undefined, trajectoryPlanningSetpoints.length * NUM_DRONES]}>
    //     <sphereGeometry args={[0.005, 4, 4]} />
    //     <meshLambertMaterial />
    //   </instancedMesh>
    // )


    const trajectoryPlanningSetpoints = [[10, 5, 0], [20, 10, 0], [30, 15, 0]];  // Example setpoints data
    const NUM_DRONES = 1;  // Number of drones

    const tempObject = new Object3D();
    const tempColour = new Color(0x00ffff);  // Cyan color

    const geometry = new SphereGeometry(0.005, 4, 4);
    const material = new MeshLambertMaterial();
    const instancedMesh = new InstancedMesh(geometry, material, trajectoryPlanningSetpoints.length * NUM_DRONES);

    function updateInstances() {
        for (let droneIndex = 0; droneIndex < NUM_DRONES; droneIndex++) {
            const positions = trajectoryPlanningSetpoints.map(x => x.slice(droneIndex * 3, (droneIndex + 1) * 3));
            positions.forEach((pos, i) => {
                const [x, y, z] = pos;
                tempObject.position.set(x, z, y);  // Adjusting coordinates for Three.js (z up)
                tempObject.updateMatrix();
                instancedMesh.setMatrixAt(i + (droneIndex * trajectoryPlanningSetpoints.length), tempObject.matrix);
                instancedMesh.setColorAt(i + (droneIndex * trajectoryPlanningSetpoints.length), tempColour);
            });
        }
        instancedMesh.instanceMatrix.needsUpdate = true;
    }

    updateInstances();


    // scene.add(instancedMesh);

    return instancedMesh;
}

