# Plan: a landing page and Getting Started, like cortex's and reflex's

Issue: #52. Status: in progress.

## Goal

The racing-drone site has the same structure as the cortex and reflex sites, so the Branes.AI sites look and read alike:

- a **splash landing page**: the hero (title, tagline, artwork, "Get Started" and "View on GitHub"), "What racing-drone is", four feature cards, the stack at a glance, where to go next, and the status;
- a **Getting Started** sidebar group first, with Introduction, Build & Test, and Repository Layout.

The site stays generated from `docs/` ([docs-site plan](../docs-site/README.md)): every page, the landing page included, is a document in `docs/`.

## Design

- **`docs/index.mdx`** is the landing page. It is MDX because the hero and the cards are Starlight components; GitHub shows it as source, and `docs/README.md` stays the folder's index on GitHub. Its front matter carries the title, description and hero, as in cortex and reflex.
- **`docs/README.md`** becomes the site's **Documentation map** page (`/overview/`), the last item of the first sidebar group.
- **`docs/getting-started/`**: `introduction.md` (the vision, the platform tiers, what this repo owns), `build-and-test.md`, `repository-layout.md`, from the top-level README. It is listed first in `docs/README.md`'s table, so it is the first sidebar group.
- **`docs-site/scripts/sync-docs.mjs`**:
  - `docs/index.mdx` becomes the home page: front matter kept (its `title` is required), an `editUrl` added, and relative Markdown links in the body rewritten as in any document;
  - each relative hero action link (`link: ./x/`) must be the route of a generated page, or the build fails;
  - with an `index.mdx`, `docs/README.md` is generated at `/overview/` instead of `/`.
- **`docs-site/src/assets/hero.svg`**: the hero artwork, in the cortex/reflex style (a 320 × 320 mark on the site's accent gradient): gates and a racing line.

## Acceptance

1. The home page renders with the splash template, the hero, the cards and the sections above, like cortex's.
2. The sidebar starts with Getting Started: Introduction, Build & Test, Repository Layout, Documentation map.
3. Every link on the landing page, including the hero actions, resolves; a broken one fails the build.
4. CI's `Docs site` job passes; after merge, the deploy publishes it.

## Departures

- The Getting Started pages repeat sections of the top-level README, as cortex's and reflex's do; a change to one should be made in the other.
