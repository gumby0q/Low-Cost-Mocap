
import pandas as pd
import matplotlib.pyplot as plt


# Load the CSV file
# csv_file_path = '/home/anatolii/projects/Low-Cost-Mocap/computer_code/api/csv_data/2024-05-24_11:04:50_seria_data.csv'


csv_file_path = '/home/anatolii/projects/Low-Cost-Mocap/computer_code/api/csv_data/2024-05-31_13:23:59_seria_data.csv' 
# csv_file_path = '/home/anatolii/projects/Low-Cost-Mocap/computer_code/api/csv_data/2024-05-31_13:25:07_seria_data.csv' 

# xy pos 0.325 0. 0.0
# -260 x trim 
# X Trim: -136 153


data = pd.read_csv(csv_file_path)
start_millis=None
end_millis=None
# start_millis=40000
# end_millis=50000

# print(data.head())
print(data['groundEffectMultiplier'].max())
print(data['groundEffectMultiplier'].min())

# Function to plot data with subplots
def plot_data(data, columns_per_subplot, start_millis=None, end_millis=None):
    plt.style.use('bmh')

    if start_millis is not None:
        data = data[data['millis'] >= start_millis]
    if end_millis is not None:
        data = data[data['millis'] <= end_millis]

    num_subplots = len(columns_per_subplot)
    fig, axes = plt.subplots(num_subplots, 1, figsize=(10, 6 * num_subplots))

    if num_subplots == 1:
        axes = [axes]

    # hack 1
    ind = 0
    for ax, columns in zip(axes, columns_per_subplot):
        for column in columns:
            data_y = data[column]
            # hack 1
            if ind == 0:
                data_y = data[column]-922

            ax.plot(data['millis'], data_y, label=column)
        # hack 1
        ind = ind + 1

        ax.legend()
        ax.set_xlabel('Millis')
        ax.set_ylabel('Value')
        ax.grid(True)

    plt.tight_layout()
    plt.show()


# Example usage
columns_per_subplot = [
    ['xPWM', 'yPWM', 'zPWM', 'yawPWM'],
    ['xPos', 'yPos', 'zPos', 'yawPos'],
    ['xVelSetpoint', 'yVelSetpoint', 'zVelSetpoint'],
    ['xVelOutput', 'yVelOutput', 'zVelOutput', 'yawPosOutput'],
    # ['groundEffectMultiplier', 'armed', 'max_t_diff_us']
    # ['groundEffectMultiplier']
]

# plot_data(data, columns_per_subplot, start_millis=40000, end_millis=50000)
plot_data(data, columns_per_subplot, start_millis=start_millis, end_millis=start_millis)

