# Contributing

Use issues for reproducible defects or focused feature proposals. Include the library revision, NCS version, board, relevant Kconfig/devicetree configuration, expected behavior, and a concise log. Distinguish hardware observations from simulated test results.

For changes:

1. Keep units, ownership, supported bounds and error recovery explicit in public headers.
2. Preserve output on failures where the API promises it. Do not hide bus failures or invent cached success.
3. Add a meaningful regression test for behavior changed by the patch.
4. Run the C and Python checks in [docs/testing.md](docs/testing.md).
5. Build affected firmware examples in NCS v3.3.0; document new wiring or SDK requirements.
6. Update the API guide and changelog when behavior or supported scope changes.

Library source uses C99; a few test doubles require C11 atomics. Match the surrounding formatting and retain SPDX license headers. New contributions are provided under Apache-2.0 unless separately agreed.

Avoid committing generated builds, firmware captures with device-specific data, SDK code/binaries, or credentials. Bench failures are useful evidence; do not describe a cross-build as a hardware pass.
