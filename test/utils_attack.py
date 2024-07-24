import torch
from utils_utils import atleast_kdim


def get_init_with_noise(model, X, y):
    init = X.clone()
    p = model(X).argmax(1)
    count = 0
    while any(p == y):
        count += 1
        print(count)
        init = torch.where(
            atleast_kdim(p == y, len(X.shape)),
            (X + 0.5*torch.randn_like(X)).clip(0, 1),
            init)
        if count > 1000:
             break
        p = model(init).argmax(1)
    return init