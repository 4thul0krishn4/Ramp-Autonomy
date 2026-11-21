## What this changes

<!-- One or two sentences. -->

## Why

<!-- The problem this solves. Link an issue if there is one. -->

## How it was verified

<!--
This is the section that matters most in this project. FSM state reporting its own success is not
verification — see docs/retrospective.md for two cases where it was actively misleading.

Please say what you actually checked: which run, against what ground truth, and what the numbers were.
-->

## Checklist

- [ ] `colcon build --symlink-install` completes with no new warnings from the packages I touched
- [ ] I verified behaviour against Gazebo ground truth, not FSM state alone, and said how above
- [ ] No raw run data (rosbags, per-topic traces, screen recordings) is included in this PR
- [ ] If I changed world geometry, I changed the generator rather than the generated block in `construction_site.sdf`
- [ ] If I added a third-party asset, I added its license and source to `THIRD_PARTY_LICENSES.md`
- [ ] I agree my contribution is provided under the Apache License 2.0 (see `LICENSE`)

## Does this change a published number?

<!-- If this affects anything in docs/results.md, say which number and what it becomes. -->
