import gaden2

def main():
    visualisation_base = gaden2.RvizVisualisationBase("gaden2")
    
    env = gaden2.EnvironmentModelPlane()
    env_visualisation = gaden2.RvizEnvironmentVisualisationPlane(visualisation_base, env)
    
    gas_src1 = gaden2.FilamentGasSource([0,0,0], 10, 10, 10)
    gas_src2 = gaden2.FilamentGasSource([10,0,1], 10, 10, 10)
    gas_sources = [gas_src1, gas_src2]
    
    gas_src_visualisation =gaden2.RvizGasSourceVisualisation(visualisation_base, gas_sources)
    
    wind = gaden2.FarrellsWindModel(env)
    
    print('Creating simulator...')
    sim = gaden2.Simulator()
    print('Creating TDLAS sensor...')
    tdlas = gaden2.OpenPathSensor(sim)
    print('Done')
    input("Press Enter to continue...")

if __name__ == '__main__':
    main()
