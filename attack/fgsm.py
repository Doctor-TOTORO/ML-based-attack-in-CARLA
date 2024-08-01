import torch
import torch.nn.functional as F

def fgsm_attack(image, epsilon, data_grad):
    sign_data_grad = data_grad.sign()
    perturbed_image = image + epsilon*sign_data_grad
    perturbed_image = torch.clamp(perturbed_image, 0, 1)
    return perturbed_image

def run_attack_cycle(data, target, num_iterations=20, epsilon=0.3):
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
        final_pred = output.max(1, keepdim=True)[1]
        if final_pred.item() != target.item():
            print("Done")
        else:
            print(max_index.item(), final_pred.item())
        data = perturbed_image.detach()
        with torch.no_grad():
            output = classify_model(data)
            target = torch.argmax(output, dim=1)
        if i == num_iterations - 1:
            return perturbed_image.squeeze()
