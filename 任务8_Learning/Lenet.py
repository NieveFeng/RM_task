
import gzip
import struct
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from torch.autograd import Variable
from torch.utils.data import TensorDataset, DataLoader
from torchvision import transforms
import math

# 定义一些超参数
use_gpu = torch.cuda.is_available()
batch_size = 256
kwargs = {'num_workers': 2, 'pin_memory': True}  # DataLoader的参数

# 读取数据的函数,先读取标签，再读取图片
def _read(image, label):
    minist_dir = '/home/nieve/feng/RM_task/任务8_Learning/data/'
    with gzip.open(minist_dir + label) as flbl:
        magic, num = struct.unpack(">II", flbl.read(8))
        label = np.frombuffer(flbl.read(), dtype=np.int8)
    with gzip.open(minist_dir + image, 'rb') as fimg:
        magic, num, rows, cols = struct.unpack(">IIII", fimg.read(16))
        image = np.frombuffer(fimg.read(), dtype=np.int8)
    return image, label

# 读取数据
def get_data():
    train_img, train_label = _read(
        'train-images-idx3-ubyte.gz',
        'train-labels-idx1-ubyte.gz')

    test_img, test_label = _read(
        't10k-images-idx3-ubyte.gz',
        't10k-labels-idx1-ubyte.gz')
    return train_img, train_label, test_img, test_label

# 获取数据
x, y, xt, yt = get_data()

train_x, train_y = [torch.LongTensor(x.reshape(-1, 1, 28, 28)).float(), 
                    torch.LongTensor(y.astype(int))]
test_x, test_y =   [torch.LongTensor(xt.reshape(-1, 1, 28, 28)).float(), 
                    torch.LongTensor(yt.astype(int))]

# 封装好数据和标签
train_dataset = TensorDataset(train_x, train_y)
test_dataset = TensorDataset(test_x, test_y)

# 定义数据加载器
train_loader = DataLoader(dataset=train_dataset,
                          shuffle=True, batch_size=batch_size, **kwargs)
test_loader = DataLoader(dataset=test_dataset,
                         shuffle=True, batch_size=batch_size, **kwargs)

# 定义lenet5
class LeNet5(nn.Module):
    def __init__(self):
        '''构造函数，定义网络的结构'''
        super().__init__()
		
        #第一层卷积和最大池化
        #输入:(6*28*28);输出：(6*14*14)
        self.conv1 = nn.Sequential(     
            #新建卷积层,kernel_size表示卷积核大小,stride表示步长,padding=2，
            #图片大小变为 28+2*2 = 32 (两边各加2列0)，保证输入输出尺寸相同
            nn.Conv2d(1,6,kernel_size=5,stride=1,padding=2), 
            nn.ReLU(),
            #可以选择取最大值池化MaxPool2d,也可以选择取平均值池化AvgPool2d,两者参数相同
            nn.MaxPool2d(kernel_size = 2 ,stride = 2,padding=0)   
        )

        #第二层卷积和最大池化
        #输入:(6*14*14);输出：(16*5*5)
        self.conv2 = nn.Sequential(
            #input_size=(6*14*14)，output_size=(16*10*10)
            nn.Conv2d(6,16,kernel_size=5,stride=1,padding=0), 
            nn.ReLU(),
            #input_size=(16*10*10)，output_size=(16*5*5)
            nn.MaxPool2d(kernel_size = 2,stride = 2,padding=0)    
        )

        #全连接层
        self.fc1 = nn.Sequential(
            nn.Linear(16*5*5,120),
            nn.ReLU()
        )
        #全连接层
        self.fc2 = nn.Sequential(
            nn.Linear(120,84),
            nn.ReLU()
        )
        #全连接层
        self.fc3 = nn.Linear(84,10)


    #网络前向传播过程
    def forward(self,x):
        x = self.conv1(x)
        x = self.conv2(x)

        #全连接层均使用的nn.Linear()线性结构，输入输出维度均为一维，故需要把数据拉为一维
        x = x.view(x.size(0), -1) 
        
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        return x


    def num_flat_features(self, x):
        # x.size()返回值为(256, 16, 5, 5)，size的值为(16, 5, 5)，256是batch_size
        size = x.size()[1:]  # x.size返回的是一个元组，size表示截取元组中第二个开始的数字
        num_features = 1
        for s in size:
            num_features *= s
        return num_features

# 参数值初始化
def weight_init(m):
    if isinstance(m, nn.Conv2d):
        n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
        m.weight.data.normal_(0, math.sqrt(2. / n))
    elif isinstance(m, nn.BatchNorm2d):
        m.weigth.data.fill_(1)
        m.bias.data.zero_()

# 训练函数
def train(epoch):
    # 调用前向传播
    model.train()
    for batch_idx, (data, target) in enumerate(train_loader):
        if use_gpu:
            data, target = data.cuda(), target.cuda()
        # 定义为Variable类型，能够调用autograd
        data, target = Variable(data), Variable(target)
        # 初始化时，要清空梯度
        optimizer.zero_grad()
        output = model(data)
        loss = criterion(output, target)
        loss.backward()
        optimizer.step()  # 相当于更新权重值
        if batch_idx % 100 == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                epoch, batch_idx * len(data), len(train_loader.dataset),
                100. * batch_idx / len(train_loader), loss.item()))

# 定义测试函数
def test():
    # 让模型变为测试模式，主要是保证dropout和BN和训练过程一致。BN是指batch normalization
    model.eval()  
    test_loss = 0
    correct = 0
    for data, target in test_loader:
        if use_gpu:
            data, target = data.cuda(), target.cuda()
        data, target = Variable(data, volatile=True), Variable(target)
        output = model(data)
        # 计算总的损失
        test_loss += criterion(output, target).item()
        pred = output.data.max(1, keepdim=True)[1]  # 获得得分最高的类别
        correct += pred.eq(target.data.view_as(pred)).cpu().sum()

    test_loss /= len(test_loader.dataset)
    print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.2f}%)\n'.format(
        test_loss, correct, len(test_loader.dataset),
        100. * correct / len(test_loader.dataset)))

# 实例化网络
model = LeNet5()
if use_gpu:
    model = model.cuda()
    print('USE GPU')
else:
    print('USE CPU')

# 定义代价函数，使用交叉熵验证
criterion = nn.CrossEntropyLoss(reduction='sum')
# 直接定义优化器，而不是调用backward
optimizer = torch.optim.Adam(model.parameters(), lr=0.001, betas=(0.9, 0.99))

# 调用参数初始化方法初始化网络参数
model.apply(weight_init)

if __name__=="__main__":
# 调用函数执行训练和测试
    for epoch in range(1, 8):
        print('\n'+'----------------start train-----------------')
        train(epoch)
        print('----------------end train-------------------'+'\n')

        print('----------------start test------------------')
        test()
        print('---------------end test---------------------')
