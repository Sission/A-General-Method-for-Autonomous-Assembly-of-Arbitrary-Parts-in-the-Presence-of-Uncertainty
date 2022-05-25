import torch
import os
from torch import nn
import torch.nn.functional as F
from torch.utils.data import DataLoader
from torchvision import datasets
from torchvision.transforms import ToTensor
import numpy as np
from sklearn import preprocessing


class NeuralNet(nn.Module):
    def __init__(self):
        super(NeuralNet, self).__init__()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(6, 16),
            nn.ReLU(),
            nn.Linear(16, 32),
            nn.ReLU(),
            nn.Linear(32, 3),
        )

    def forward(self, x):
        # x = self.flatten(x)
        model = self.linear_relu_stack(x)
        return model


class ClassificationNet(nn.Module):
    def __init__(self):
        super(ClassificationNet, self).__init__()
        # This applies Linear transformation to input data.
        self.fc1 = nn.Linear(3, 3)

        # This applies linear transformation to produce output data
        self.fc2 = nn.Linear(3, 3)

    # This must be implemented
    def forward(self, x):
        # Output of the first layer
        x = self.fc1(x)
        # Activation function is Relu. Feel free to experiment with this
        x = torch.tanh(x)
        # This produces output
        x = self.fc2(x)
        return x

    # This function takes an input and predicts the class, (0 or 1)
    def predict(self, x):
        # Apply softmax to output.
        pred = F.softmax(self.forward(x), dim=1)
        ans = []
        # Pick the class with maximum weight
        for t in pred:
            if t[0] > t[1]:
                ans.append(0)
            else:
                ans.append(1)
        return torch.tensor(ans)


def read_force():
    file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'rt_data/franka_force_data.csv')
    print(file_path)
    data = np.loadtxt(file_path, delimiter=',', dtype=np.float64, skiprows=1)

    uncertainty = data[:, 0:3]
    fm = data[:, 3:6]
    tm = data[:, 6:9]
    print('MAX Fx', np.amax(fm[:, 0]))
    print('MAX Fy', np.amax(fm[:, 1]))
    print('MAX Fz', np.amax(fm[:, 2]))
    print('MIN Fx', np.amin(fm[:, 0]))
    print('MIN Fy', np.amin(fm[:, 1]))
    print('MIN Fz', np.amin(fm[:, 2]))

    normalized_fm = preprocessing.normalize(fm)
    normalized_tm = preprocessing.normalize(tm)
    normalized_x, normalized_y, normalized_z = np.hsplit(normalized_fm, 3)

    idx_zone1 = np.flatnonzero((normalized_x > 0) & (normalized_y > 0))
    idx_zone2 = np.flatnonzero((normalized_x > 0) & (normalized_y < 0))
    idx = [idx_zone1, idx_zone2]

    y = np.zeros_like(normalized_x)
    # d2_plot(normalized_x, normalized_y)

    y[idx_zone1] = 1
    y.reshape((-1,))

    return uncertainty, y


def predict(model, x):
    x = torch.from_numpy(x).type(torch.FloatTensor)
    ans = model.predict(x)
    return ans.numpy()


def direction_info(fm):
    fm = np.asarray(fm).reshape((1, 3))
    file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'model/classification_model.pth')
    loaded_model = torch.load(file_path)
    loaded_model.eval()
    direction = predict(model=loaded_model, x=fm)
    if direction.item() == 1:
        print('In the Zone Fx > 0, Fy > 0')
        return 1
    else:
        print('In the Zone Fx > 0, Fy < 0')
        return 0


def uncertainty_pred(zone_info, fm):
    fm = np.asarray(fm).reshape((1, 6))
    fm = torch.from_numpy(fm).type(torch.DoubleTensor)
    file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), f'model/model_zone{zone_info}.pth')
    loaded_model = torch.load(file_path)
    loaded_model.eval()
    prediction = loaded_model.forward(fm)
    prediction = prediction.cpu().detach().numpy().tolist()[0]
    return prediction


if __name__ == "__main__":
    # direction_info([5.05578069, 0.55216538, -9.42291467])
    read_force()
