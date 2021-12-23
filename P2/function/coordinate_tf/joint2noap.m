function T6 = joint2noap(theta, rb) % joints -> noap
T6 = rb.forward(theta);
end