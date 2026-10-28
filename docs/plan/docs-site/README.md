# Plan: publish ./docs as a GitHub Pages site

Issue: #48. Status: in progress.

## Goal

A published site for this repo's documentation at <https://branes-ai.github.io/racing-drone/>, set up like reflex's and cortex's (Astro Starlight, a Doxygen C++ API reference, deployed by GitHub Actions). One difference from them: here the site is **always generated from `docs/`**. Nothing is authored in `docs-site/`; the documents we write and review stay in `docs/`, where GitHub renders them too.

## Design

- **`docs-site/`**: Starlight, with the same Astro and Starlight versions as reflex and cortex, and the same base-path link rewriting for Pages.
- **`docs-site/scripts/sync-docs.mjs`** generates `docs-site/src/content/docs/` (gitignored) from `docs/`, before every `dev` and `build`:
  - every `docs/**/*.md` becomes a page; a folder's `README.md` becomes that folder's index, and `docs/README.md` the home page;
  - the page title is the document's first `# ` heading, which is removed from the body (Starlight renders the title);
  - each page's "Edit page" link opens its source in `docs/`;
  - a relative link to another document in `docs/` becomes a link to its page, with the anchor kept;
  - a relative link to any other file in the repo (source, data, the top-level README) becomes a link to that file on GitHub, at `main`;
  - **a relative link whose target does not exist fails the build**, naming the file and line. The site cannot publish a broken link that GitHub would also show broken.
- **The sidebar** is generated too: one group per top-level folder of `docs/`, in the order of the table in `docs/README.md`; within a group, the folder's index first, then the pages in file-name order, labeled by title.
- **The API reference**: Doxygen over `sdk/include`, `sim/include` and `baselines/glue/include`, at `<base>api/`.
- **Deploy**: `.github/workflows/deploy-docs.yml`, copied from reflex's without the WebAssembly steps. It runs on pushes to main that touch `docs/`, `docs-site/`, the public headers, or the workflow.
- **CI**: a `Docs site` job in `ci.yml` builds the site on every PR, so a broken link fails the PR instead of the deploy after merge.

## Acceptance

1. `npm run build` in `docs-site/` builds a page for every `docs/**/*.md`, and the count matches.
2. Every relative link in the generated pages resolves: to a generated page, or to an existing file on GitHub.
3. A deliberately broken relative link in `docs/` fails `npm run build` with the file and line.
4. CI's `Docs site` job passes on the PR.
5. After merge, the deploy workflow publishes the site (requires the one-time Pages setting below).

## One-time setting

Settings → Pages → Build and deployment → Source = "GitHub Actions". A repo admin sets it once (or `gh api -X POST repos/branes-ai/racing-drone/pages -f build_type=workflow`).

## Departures from reflex and cortex

- Content is generated from `docs/`, not authored in `docs-site/src/content/docs/`.
- No KaTeX: no document in `docs/` uses math yet, and `remark-math` would read a dollar amount in prose as math. Add it with the first document that needs it.
- No WebAssembly widgets or traces.
