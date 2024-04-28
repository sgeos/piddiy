<!-- README.md -->

# PIDDIY

## Philosophy

PIDDIY (Proportional-Integral-Derivative Do It Yourself) is designed to
give users the tools to build their own PID controllers. The library allows
manual control over the three primary components of PID systems:

- **Proportional (P)**: Adjusts the controller output proportionally to the error.
- **Integral (I)**: Accounts for past errors, integrating them over time to eliminate residual errors.
- **Derivative (D)**: Predicts future errors, focusing on the rate of change of the error.

It includes a few standard implementations for these calculations to address
common control scenarios. Users requiring more complex behavior are encouraged
to implement their own compute functions. The library intentionally omits
features like output limits and automatic integral windup resolution to
maintain simplicity and user-directed customization.

## License

This library is licensed under the BSD Zero Clause License (0BSD) to make the
code available in a public-domain-equivalent capacity, ensuring unrestricted use.

