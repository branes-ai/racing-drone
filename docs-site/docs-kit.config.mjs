// The docs site's configuration: everything docs-kit does not default.
// https://github.com/branes-ai/docs-kit
export default {
  repo: 'racing-drone',
  title: 'Racing Drone',
  description: 'Branes.AI mission and flight controller for an autonomous racing drone: perception, planning and optimal control from the Branes.AI platform, assembled into a vehicle that flies a race course on its own.',
  accent: {
    dark: { low: '#0c3a34', accent: '#21c7b0', high: '#b4f5ea' },
    light: { low: '#c9f3ec', accent: '#0b7d6e', high: '#053b34' },
  },
  api: {
    name: 'Racing Drone C++ API',
    brief: 'branes::racing — racing\'s contracts, slots, stack, simulator and scoreboard',
    input: ['sdk/include', 'sim/include'],
  },
};
