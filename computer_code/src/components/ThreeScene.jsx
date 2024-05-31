import React, { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import * as math from "mathjs";
import { useThreeDataStore } from "./ThreeDataStore"
import ThreeCameraWireframe from './ThreeCameraWireframe';
import TreePoints from './TreePoints';

// {cameraPoses.map(({ R, t }, i) => (
//   <CameraWireframe R={R} t={t} toWorldCoordsMatrix={toWorldCoordsMatrix} key={i} />
// ))}
// <Points objectPointsRef={objectPoints} objectPointErrorsRef={objectPointErrors} count={objectPointCount} />
// <Objects filteredObjectsRef={filteredObjects} count={objectPointCount} />
// <TrajectoryPlanningSetpoints trajectoryPlanningSetpoints={trajectoryPlanningSetpoints} NUM_DRONES={NUM_DRONES} />


const ThreeScene = () => {
    const mountRef = useRef(null);

    const setScene = useThreeDataStore(state => state.setScene);

    useEffect(() => {
        if (mountRef.current) {
            // Scene setup
            const scene = new THREE.Scene();
            
            // <Canvas orthographic camera={{ zoom: 1000, position: [0, 0, 10] }}>
            const camera = new THREE.PerspectiveCamera(/* fov */75, /* aspect ratio */ 1, /* near */ 0.1, 1000);
            const renderer = new THREE.WebGLRenderer();
            // renderer.setSize(window.innerWidth, window.innerHeight);
            renderer.setPixelRatio( window.devicePixelRatio );
            renderer.setSize(1200, 800);
            mountRef.current.appendChild(renderer.domElement);

            // Adding OrbitControls
            const controls = new OrbitControls(camera, renderer.domElement);
            // controls.enableDamping = true; // Optional: this enables damping (inertia), which can provide a smoother control experience.
            // controls.dampingFactor = 0.05;

            // Ambient Light
            // const ambientLight = new THREE.AmbientLight(0x404040); 
            // const ambientLight = new THREE.AmbientLight(); 
            // scene.add(ambientLight);

            // Directional Light
            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
            // directionalLight.position.set(1, 1, 1);
            directionalLight.position.set( 0, 20, 10 );
            // scene.add(directionalLight);


            // camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.25, 100 );
            // camera.position.set( - 5, 3, 10 );
            // camera.lookAt( 0, 2, 0 );

            // scene = new THREE.Scene();
            scene.background = new THREE.Color( 0xe0e0e0 );
            // scene.fog = new THREE.Fog( 0xe0e0e0, 20, 100 );

            // clock = new THREE.Clock();

            // lights

            const hemiLight = new THREE.HemisphereLight( 0xffffff, 0x8d8d8d, 3 );
            hemiLight.position.set( 0, 20, 0 );
            scene.add( hemiLight );

            const dirLight = new THREE.DirectionalLight( 0xffffff, 3 );
            dirLight.position.set( 0, 20, 10 );
            scene.add( dirLight );


            // Axes Helper
            const axesHelper = new THREE.AxesHelper(5);
            // const axesHelper = new THREE.AxesHelper(0.2);
            scene.add(axesHelper);

            // Grid Helper
            // const gridHelper = new THREE.GridHelper(10, 10);
            const gridHelper = new THREE.GridHelper(4, 4 * 10);
            scene.add(gridHelper);


            // Mock data
            const objectPoints = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]; // Example data
            const objectPointErrors = [0.1, 0.5, 0.9]; // Example error values
            // 
            // const geometry = new THREE.SphereGeometry(0.008, 4, 4);
            const geometry = new THREE.SphereGeometry(0.008, 20, 20);
            const material = new THREE.MeshLambertMaterial();
            const instancedMesh = new THREE.InstancedMesh(geometry, material, objectPoints.length);

            scene.add(instancedMesh);

            // Helper functions
            const maxError = objectPointErrors.length !== 0 ? math.max(objectPointErrors) : 1;

            const errorToColour = (error) => {
                const scaledError = error / maxError;
                const logError = scaledError / (0.1 + scaledError);
                const tempColour = new THREE.Color(0x009999 + Math.round(logError * 0xff) * 0x10000);
                return tempColour;
            };

            const tempObject = new THREE.Object3D();
            objectPoints.forEach(([x, y, z], i) => {
                tempObject.position.set(x, z, y); // swapping y and z to match Three.js coordinate system
                tempObject.updateMatrix();
                instancedMesh.setMatrixAt(i, tempObject.matrix);
                instancedMesh.setColorAt(i, errorToColour(objectPointErrors[i]));
            });


            instancedMesh.instanceMatrix.needsUpdate = true;
            // Adding a cube
            // const geometry = new THREE.BoxGeometry();
            // const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
            // const cube = new THREE.Mesh(geometry, material);
            // scene.add(cube);


            // renderer.setClearColor(0xeeeeee); // Set a light grey background

            // Animation loop
            const animate = function () {
                requestAnimationFrame(animate);
                
                // Rotate the cube
                // cube.rotation.x += 0.01;
                // cube.rotation.y += 0.01;

                renderer.render(scene, camera);
            };

            animate();

            camera.position.z = 5;


            setScene(scene);

            // Cleanup function
            return () => {
                renderer.dispose();
                if (mountRef.current) {
                    mountRef.current.removeChild(renderer.domElement);
                }
            };
        }
    }, [setScene]);

    const cameraPoses = useThreeDataStore(state => state.cameraPoses);
    const scene = useThreeDataStore(state => state.scene);
    const toWorldCoordsMatrix = useThreeDataStore(state => state.toWorldCoordsMatrix);
    useEffect(() => {
        let segments = [];
        cameraPoses.forEach(pose => {
            // console.log("pose", pose)
            const lineSegments = ThreeCameraWireframe({
                ...pose,
                toWorldCoordsMatrix,
            });
            segments.push(lineSegments)
        })
        segments.forEach(items => {
            scene.add(items);
        })

        return () => {
            segments.forEach(items => {
                scene.remove(items);
            })
        }
    }, [scene, cameraPoses, toWorldCoordsMatrix]);

    const objectPoints = useThreeDataStore(state => state.objectPoints);
    const objectPointErrors = useThreeDataStore(state => state.objectPointErrors);
    useEffect(() => {
        // if (scene) {
        //     let segments = TreePoints({ objectPoints, objectPointErrors });

        //     // cameraPoses.forEach(pose => {
        //     //     // console.log("pose", pose)
        //     //     const lineSegments = ThreeCameraWireframe({
        //     //         ...pose,
        //     //         toWorldCoordsMatrix,
        //     //     });
        //     //     segments.push(lineSegments)
        //     // })
        //     // segments.forEach(items => {
        //     //     scene.add(items);
        //     // })
        //     scene.add(segments);
    
        //     return () => {
        //         // segments.forEach(items => {
        //         //     scene.remove(items);
        //         // })
        //         scene.remove(segments);
        //     }
        // }

    }, [scene, objectPoints, objectPointErrors]);

    console.log("ThreeScene");


    return <div ref={mountRef} />;
};

export default ThreeScene;
