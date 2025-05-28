from src.Simulator import Simulator

if __name__ == '__main__':
    sim_config_file = "xml/nfs.xml"
    trace = True
    verbose = True

    forced_load = 400
    num_simulations = 1

    Simulator(sim_config_file, trace, verbose, forced_load, num_simulations)
    # min_load = 75
    # max_load = 400
    # step = 25
    #
    # for forced_load in range(min_load, max_load + 1, step):
    #     Simulator(sim_config_file, trace, verbose, forced_load, num_simulations)

