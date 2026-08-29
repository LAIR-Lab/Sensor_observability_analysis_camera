## Describing SOA and implimenting SOA for uni-directional sensors/Camera.

## SOA

$$
T_{\text{uni}}(\hat{\mathbf{s}},\mathbf{p}_{\mathrm{poi}})=
\left(
\frac{\theta^{i}_{\mathrm{FOV}}-\theta^{i}}
{\theta^{i}_{\mathrm{FOV}}}
\right)
$$

$$
s^{i}=
left(
0,\frac{\overline{\mathrm{FOV}}^{i}-\theta^{i}}
{\overline{\mathrm{FOV}}^{i}}
\right)
$$

$$
\tilde{\mathbf{s}}^{i}=
left(
0,\,
T_{\text{uni}}(\hat{\mathbf{s}},\mathbf{p}_{\mathrm{poi}})
\-
\textstyle\sum_i \lambda_i
T_{\text{uni}}(\mathbf{p}_{\mathrm{poi}},\mathbf{p}_{\mathrm{obs}})
\right)
$$
