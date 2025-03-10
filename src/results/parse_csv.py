# External
import csv
import numpy as np
import matplotlib.pyplot as plt


CSV_PATH = 'boat_log.csv'
OUT_PATH = 'csv_plot.pdf'


def parse_csv() -> None:
    x, x_d, u, u_a, t = [], [], [], [], []

    # Read CSV
    with open(CSV_PATH, 'r') as log_file:
        log_reader = csv.reader(log_file)

        # Parse CSV
        for i, row in enumerate(log_reader):
            if i == 0: continue
            row = [float(i) for i in row]
            x.append(row[:6])
            x_d.append(row[6:12])
            u.append(row[12:14])
            u_a.append(row[14:16])
            t.append(row[-1])

    # Plot results
    t   = np.asarray(t)
    x   = np.asarray(x).T
    x_d = np.asarray(x_d).T
    u   = np.asarray(u).T
    u_a = np.asarray(u_a).T

    num_states, num_inputs = 6, 2

    plt.rc('savefig', format='pdf')
    plt.rc('savefig', bbox='tight')

    ylabels_states = ['x [m]', 
                      'y [m]', 
                      '\u03B8 [deg]',    # theta
                      '\u03B3 [deg]',    # gamma
                      '\u03C6 [deg]',    # phi
                      '\u03B7 [deg]']    # eta
    ylabels_inputs = ['\u03C3 [deg/s]',  # sigma
                      '\u03C9 [deg/s]']  # omega
    
    idx_map = {1 : 1, 2 : 3, 3 : 5, 4 : 7, 5 : 2, 6 : 4, 7 : 6, 8 : 8}
    
    fig1 = plt.figure(1)
    fig1.set_figheight(8.0)
    fig1.set_figwidth(10.0)

    for i in range(num_states):
        ax2 = plt.subplot(4, 2, idx_map[i+1])  # 4 rows, 2 columns, index i + 1
        if i == 4:  # Phi
            plt.plot(t, x[i, :]  *180.0/np.pi, 'C0'  , label='Estimated')
            plt.plot(t, u_a[1, :]*180.0/np.pi, 'C1--', label='Input')
            plt.plot(t, x_d[i, :]*180.0/np.pi, 'C2--', label='Desired')
        elif i == 5:  # Eta
            plt.plot(t, x[i, :]  *180.0/np.pi, 'C0'  , label='Estimated')
            plt.plot(t, u_a[0, :]*180.0/np.pi, 'C1--', label='Inpit')
            plt.plot(t, x_d[i, :]*180.0/np.pi, 'C2--', label='Desired')
        elif 'deg' in ylabels_states[i]:
            plt.plot(t, x[i, :]  *180.0/np.pi, 'C0'  , label='Estimated')
            plt.plot(t, x_d[i, :]*180.0/np.pi, 'C2--', label='Desired')
        else:
            plt.plot(t   , x[i, :]  , 'C0'  , label='Estimated')
            plt.plot(t[0], x[i, 0]  , 'C1--', label='Input')  # Trick to get legend right, not real data
            plt.plot(t   , x_d[i, :], 'C2--', label='Desired')
        plt.grid(color='0.95')
        plt.ylabel(ylabels_states[i])
        if i == 0:
            plt.legend()
        if i == 3:
            plt.xlabel('t [s]')
        else:
            plt.setp(ax2, xticklabels=[])
    for i in range(num_inputs):
        ax2 = plt.subplot(4, 2, idx_map[num_states+i+1])  # 4 rows, 2 columns, index after state plots
        if 'deg' in ylabels_inputs[i]:
            plt.step(t, u[i, :]*180.0/np.pi , 'C0'  , label='Estimated')
            plt.plot(t, np.zeros(u.shape[1]), 'C2--', label='Desired')
        else:
            plt.step(t, u[i, :]             , 'C0'  , label='Estimated')
            plt.plot(t, np.zeros(u.shape[1]), 'C2--', label='Desired')
        plt.ylabel(ylabels_inputs[i])
        plt.grid(color='0.95')
        if i != num_inputs-1:
            plt.setp(ax2, xticklabels=[])  
    plt.xlabel('t [s]')
    plt.savefig(OUT_PATH)
    plt.show()


if __name__ == '__main__':
    parse_csv()
