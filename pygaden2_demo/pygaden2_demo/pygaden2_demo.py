import gaden2

def main():
    visualisation_base = gaden2.RvizVisualisationBase("gaden2")
    
    env = gaden2.EnvironmentModelPlane()
    env_visualisation = gaden2.RvizEnvironmentVisualisationPlane(visualisation_base, env)
    
    print('Creating simulator...')
    sim = gaden2.Simulator()
    print('Creating TDLAS sensor...')
    tdlas = gaden2.OpenPathSensor(sim)
    print('Done')
    input("Press Enter to continue...")

if __name__ == '__main__':
    main()
