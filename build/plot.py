import argparse
import matplotlib.pyplot as plt

def plot_data_to_png(dataL, dataR,filename):
    # Define the desired size in pixels
    desired_width_px = 800
    desired_height_px = 600

    # Calculate the corresponding size in inches based on the DPI
    dpi = 100
    desired_width_in = desired_width_px / dpi
    desired_height_in = desired_height_px / dpi

    # Plot all columns on the same subplot with separate y-axes
    fig, axs = plt.subplots(2, 2, sharex=True,figsize=(desired_width_in, desired_height_in))
    fig.tight_layout()
    # LEFT
    axs[0,0].plot(dataL['OUT'], label='OUT', color='blue')
    axs[0,0].set_title('Left')
    ax2l = axs[0,0].twinx()
    ax2l.plot(dataL['ENC'], label='ENC', color='green')
    # axs[] = ax2.twinx()
    axs[1,0].plot(dataL['IN'], label='IN', color='red')
    # ax3 = axs[1].twinx()
    axs[1,0].plot(dataL['SET'], label='SET', color='black')

    # Set plot title and labels
    axs[0,0].set_ylabel("OUT")
    axs[0,0].yaxis.label.set_color('blue')
    ax2l.set_ylabel("ENC")
    ax2l.yaxis.label.set_color('green')
    axs[1,0].set_ylabel("IN & SET")

    #RiGHT
    axs[0,1].plot(dataR['OUT'], label='OUT', color='blue')
    axs[0,1].set_title('Right')
    ax2r = axs[0,1].twinx()
    ax2r.plot(dataR['ENC'], label='ENC', color='green')
    # axs[] = ax2.twinx()
    axs[1,1].plot(dataR['IN'], label='IN', color='red')
    # ax3 = axs[1].twinx()
    axs[1,1].plot(dataR['SET'], label='SET', color='black')

    # Set plot title and labels
    axs[0,1].set_ylabel("OUT")
    axs[0,1].yaxis.label.set_color('blue')
    ax2r.set_ylabel("ENC")
    ax2r.yaxis.label.set_color('green')
    axs[1,1].set_ylabel("IN & SET")

    # plt.rcParams["figure.figsize"] = (30,3)
    # plt.title("Motor PID Data")
    plt.xlabel("Sample")
    # Show legend
    plt.legend()

    # Save the plot to a PNG file
    # plt.savefig("motorPid_plot.png", dpi=dpi)
    plt.savefig(filename, dpi=dpi)

    # Close the plot to release resources (optional)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description='Generate a plot and save it to a PNG file.')
    parser.add_argument('filename', type=str, help='Name of the PNG file to be created')
    parser.add_argument('--limit', type=int, default=None, help='Limit the number of rows to read from files')
    args = parser.parse_args()

    # Read the CSV file into lists
    columns = []
    dataL = { 'OUT': [], 'ENC': [], 'IN': [], 'SET': [] }
    dataR = { 'OUT': [], 'ENC': [], 'IN': [], 'SET': [] }

    with open("motorLeftPid.csv", 'r') as file_l:
        header = file_l.readline().strip().split(',')
        columns = [col.strip() for col in header]

        for i, line in enumerate(file_l):
            if args.limit is not None and i >= args.limit:
                break

            values = line.strip().split(',')
            dataL['OUT'].append(float(values[0]))
            dataL['ENC'].append(int(values[1]))
            dataL['IN'].append(float(values[2]))
            dataL['SET'].append(float(values[3]))

    with open("motorRightPid.csv", 'r') as file_r:
        header = file_r.readline().strip().split(',')
        columns = [col.strip() for col in header]

        for i, line in enumerate(file_r):
            if args.limit is not None and i >= args.limit:
                break

            values = line.strip().split(',')
            dataR['OUT'].append(float(values[0]))
            dataR['ENC'].append(int(values[1]))
            dataR['IN'].append(float(values[2]))
            dataR['SET'].append(float(values[3]))

    plot_data_to_png(dataL, dataR, args.filename)

if __name__ == '__main__':
    main()