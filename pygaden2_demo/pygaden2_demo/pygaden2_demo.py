import time

import gaden2

def main():
    visualisation_base = gaden2.RvizVisualisationBase("gaden2")
    
    env = gaden2.EnvironmentModelPlane()
    env_visualisation = gaden2.RvizEnvironmentVisualisationPlane(visualisation_base, env)
    
    methane = gaden2.Methane()
    gas_src1 = gaden2.FilamentGasSource(position = [-45,0,1],
                                        gas = methane,
                                        release_rate = 10, # [kg/h]
                                        filament_spawn_radius = 0.0)
    gas_src2 = gaden2.FilamentGasSource(position = [0,10,3],
                                        gas = methane,
                                        release_rate = 10, # [kg/h]
                                        filament_spawn_radius = 0.0)
    gas_sources = [gas_src1]
    gas_src_visualisation = gaden2.RvizGasSourceVisualisation(visualisation_base, gas_sources)
    
    wind = gaden2.FarrellsWindModel(env,
                                    noise_gain = 2.0,
                                    noise_damp = 0.1,#900004,
                                    noise_bandwidth = 0.2)
    wind_visualisation = gaden2.RvizWind2dVisualisation(visualisation_base,
                                                        wind,
                                                        resolution = 2.0)
    
    gas_model = gaden2.FilamentModel(env,
                                     wind,
                                     gas_sources,
                                     filament_noise_std = 0.1)
    gas_model_visualisation = gaden2.RvizFilamentVisualisation(visualisation_base, gas_model)
    
    print('Creating simulator...')
    dt = 0.2
    sim = gaden2.Simulator(gas_model, dt = dt)
    
    try:
        while True:
            sim.increment()
            #time.sleep(dt/4)
    except KeyboardInterrupt:
        print('Catched Ctrl+C. Will end.')
    
    #print('Creating TDLAS sensor...')
    #tdlas = gaden2.OpenPathSensor(sim)
    #print('Done')
    #input("Press Enter to continue...")

if __name__ == '__main__':
    main()
