import torch
import torch.nn.functional as F

def fgsm_attack(image, epsilon, data_grad):
    data_grad = data_grad.cuda()
    sign_data_grad = data_grad.sign()
    perturbed_image = image + epsilon*sign_data_grad
    perturbed_image = torch.clamp(perturbed_image, 0, 1)
    return perturbed_image

def run_attack_cycle(data, target, classify_model, num_iterations=10, epsilon=0.3):
    data = data.cuda()
    target = target.cuda()
    for i in range(num_iterations):
        data.requires_grad = True
        output = classify_model(data)
        init_pred = output.max(1, keepdim=True)[1]
        loss = F.nll_loss(output, target)
        classify_model.zero_grad()
        loss.backward()
        data_grad = data.grad.data
        perturbed_image = fgsm_attack(data, epsilon, data_grad)
        output = classify_model(perturbed_image)
        data = perturbed_image.detach()
        with torch.no_grad():
            output = classify_model(data)
            target = torch.argmax(output, dim=1)
        if i == num_iterations - 1:
            return perturbed_image.squeeze().detach()
