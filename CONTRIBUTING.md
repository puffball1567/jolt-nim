# Contributing

Contributions are welcome through pull requests. Keep each pull request focused
and explain the user-visible behavior it changes.

## Development setup

The binding targets Jolt Physics 5.6.0 and requires Nim 2.0 or newer, a C++17
compiler, Jolt headers, and a matching static Jolt library. Register a checkout
locally with:

```sh
nimble develop
```

Jolt's compile definitions and CPU feature flags must be identical for the
library and the Nim C++ translation unit. See the README's Build section for a
complete example.

## Testing

Run the native suite against the Jolt checkout and library you built:

```sh
tests/run_jolt_tests.sh <path-to-jolt> <path-to-libJolt.a>
```

Validate package metadata separately:

```sh
nimble check
```

Changes to `jolt/raw` should also update and run the applicable callable API
audit or backend compile check under `tests/`. New ownership-safe behavior
should include a native regression test. Renderer-specific changes should be
exercised by the corresponding example where practical.

## Pull requests

- Add tests for new behavior and bug fixes.
- Preserve deterministic cleanup and avoid invoking Nim callbacks from Jolt
  worker threads.
- Keep the public API documented in the README or module comments.
- Note compatibility changes explicitly.
- Confirm that contributed code can be distributed under the project's MIT
  License.
