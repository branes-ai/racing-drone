// @ts-check
import fs from 'node:fs';
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import rehypeRewrite from 'rehype-rewrite';

// The content and the sidebar are generated from ../docs by
// scripts/sync-docs.mjs, which the predev/prebuild hooks run first.
const sidebarFile = new URL('./src/generated/sidebar.json', import.meta.url);
if (!fs.existsSync(sidebarFile)) {
  throw new Error('src/generated/sidebar.json is missing: run `npm run sync` (scripts/sync-docs.mjs) first');
}
const docsSidebar = JSON.parse(fs.readFileSync(sidebarFile, 'utf8'));

// Deployment configuration.
//   For GitHub Pages: DEPLOY_TARGET=github-pages (sets the project base path).
//   For local dev / other hosts: no env vars needed.
const isGitHubPages = process.env.DEPLOY_TARGET === 'github-pages';
const site = isGitHubPages
  ? 'https://branes-ai.github.io'
  : (process.env.SITE_URL || 'http://localhost:4321');
const base = isGitHubPages ? '/racing-drone/' : '/';

// The Doxygen C++ API reference is generated into public/api/ and served at
// <base>api/. Starlight prepends `base` to internal sidebar links, so this must
// be base-relative (prefixing it here too would double it).
const apiHref = '/api/';

// Rehype plugin to rewrite absolute internal links with the base path, so the
// root-relative links sync-docs.mjs generates resolve under /racing-drone/ on
// Pages. Protocol-relative `//host` links and already-prefixed links are left
// untouched.
const basePrefix = base.replace(/\/$/, ''); // '/racing-drone'
const rehypeBaseLinks = isGitHubPages ? [
  [
    rehypeRewrite,
    {
      rewrite: (node) => {
        if (node.type !== 'element') return;
        const attr = node.tagName === 'a' ? 'href' : node.tagName === 'img' ? 'src' : null;
        if (!attr || typeof node.properties?.[attr] !== 'string') return;
        const val = node.properties[attr];
        const isRootRelative = val.startsWith('/') && !val.startsWith('//');
        const alreadyPrefixed = val === basePrefix || val.startsWith(basePrefix + '/');
        if (isRootRelative && !alreadyPrefixed) {
          node.properties[attr] = basePrefix + val;
        }
      },
    },
  ],
] : [];

// https://astro.build/config
export default defineConfig({
  server: { host: '0.0.0.0' },
  site,
  base,
  trailingSlash: 'always',
  markdown: {
    rehypePlugins: [...rehypeBaseLinks],
  },
  integrations: [
    starlight({
      title: 'Racing Drone',
      description: 'Branes.AI mission and flight controller for an autonomous racing drone: perception, planning and optimal control from the Branes.AI platform, assembled into a vehicle that flies a race course on its own.',
      favicon: '/favicon.svg',
      social: [
        { icon: 'github', label: 'GitHub', href: 'https://github.com/branes-ai/racing-drone' },
      ],
      // Each generated page sets its own editUrl, to its source in docs/.
      editLink: {
        baseUrl: 'https://github.com/branes-ai/racing-drone/edit/main/docs/',
      },
      sidebar: [
        ...docsSidebar,
        {
          label: 'API Reference',
          items: [
            { label: 'C++ API (Doxygen)', link: apiHref, attrs: { target: '_blank' } },
          ],
        },
      ],
      customCss: ['./src/styles/custom.css'],
    }),
  ],
});
