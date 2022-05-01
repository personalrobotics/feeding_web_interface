import create from "zustand";
import { persist } from "zustand/middleware"
import * as constants from './Constants';


const useStore = create(persist((set) => ({
    defaultState: { feeding_status: constants.States[7] },

    changeState: (variable) =>
        set(() => ({
            defaultState: 
                { feeding_status: variable },
        })),
})));


export default useStore;