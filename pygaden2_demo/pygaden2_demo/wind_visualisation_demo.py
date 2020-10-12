import gaden2
import time

def main():
    visualisation_base = gaden2.RvizVisualisationBase("gaden2")
    
    env = gaden2.EnvironmentModelPlane()
    env_visualisation = gaden2.RvizEnvironmentVisualisationPlane(visualisation_base, env)
    
    wind = gaden2.FarrellsWindModel(env)
    
    windvisu = gaden2.RvizWind2dVisualisation(visualisation_base, wind)
    
    try:
        while True:
            wind.increment(0.1, 0)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('Catched Ctrl+C. Will end.')
    
    #input("Press Enter to continue...")

if __name__ == '__main__':
    main()
