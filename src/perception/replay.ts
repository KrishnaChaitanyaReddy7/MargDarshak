import { World } from '../core/types';
import { createWorld, stepWorld } from '../planning/world';

/**
 * Loads recorded sensor replay frames from a URL or generates
 * pre-recorded synthetic playback worlds.
 *
 * Exact Contract Signature:
 * loadReplay(url: string): Promise<World[]>
 *
 * @param url Resource URL or scenario descriptor for playback
 * @returns Array of World states representing time snapshots
 */
export async function loadReplay(url: string): Promise<World[]> {
  // If fetching from a real JSON endpoint:
  if (url.startsWith('http://') || url.startsWith('https://') || url.endsWith('.json')) {
    try {
      const response = await fetch(url);
      if (response.ok) {
        const data = await response.json();
        if (Array.isArray(data)) {
          return data as World[];
        }
      }
    } catch {
      // Fallback to generated synthetic frames if fetch fails
    }
  }

  // Generate synthetic replay sequence for offline demonstration
  const frames: World[] = [];
  const baseWorld = createWorld('dense_mixed');
  const dt = 1 / 60;
  const numFrames = 300; // 5 seconds of replay

  let currentWorld = JSON.parse(JSON.stringify(baseWorld)) as World;

  for (let f = 0; f < numFrames; f++) {
    stepWorld(currentWorld, dt);
    frames.push(JSON.parse(JSON.stringify(currentWorld)));
  }

  return frames;
}
