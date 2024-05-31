import { create } from 'zustand'
// import ThreeCameraWireframe from './ThreeCameraWireframe'
// import { LineSegments } from 'three'


interface ThreeDataState {
    NUM_DRONES: number
    // cameraWireFrames: LineSegments[]
    // increase: (by: number) => void
    // setDataParams: (camrasParams: { R: Array<Array<number>>, t: Array<number>, toWorldCoordsMatrix: number[][] }[]) => void

    scene: THREE.Scene | null
    setScene: (scene: THREE.Scene | null) => void


    objectPoints: number[][][]
    objectPointErrors: number[][]
    filteredObjects: Array<{pos:[]}[]>
    cameraPoses: Array<{ R:number[][], t:number[] }>
    trajectoryPlanningSetpoints: number[][]
    toWorldCoordsMatrix: number[][]

    setObjectPoints: (points: number[][][]) => void
    setObjectPointErrors: (points: number[][]) => void
    setFilteredObjects: (points: Array<{pos:[]}[]>) => void
    setCameraPoses: (poses: Array<{ R:number[][], t:number[] }>) => void
    setTrajectoryPlanningSetpoints: (data: number[][]) => void
    setToWorldCoordsMatrix: (data: number[][]) => void
}

const useThreeDataStore = create<ThreeDataState>()((set) => ({
    NUM_DRONES: 0,
    scene: null,
    cameraWireFrames: [],
    
    setScene: (scene) => {
        set({ scene: scene });
    },


    objectPoints: [],
    setObjectPoints: (points) => {
        set({ objectPoints: points });
    },
    objectPointErrors: [],
    setObjectPointErrors: (points) => {
        set({ objectPointErrors: points });
    },
    filteredObjects: [],
    setFilteredObjects: (points) => {
        set({ filteredObjects: points });
    },
    cameraPoses: [],
    setCameraPoses: (poses) => {
        set({ cameraPoses: poses });
    },
    trajectoryPlanningSetpoints: [],
    setTrajectoryPlanningSetpoints: (points) => {
        set({ trajectoryPlanningSetpoints: points });
    },

    toWorldCoordsMatrix: [],
    setToWorldCoordsMatrix: (data) => {
        set({ toWorldCoordsMatrix: data });
    },
}))



export {
    useThreeDataStore
};

