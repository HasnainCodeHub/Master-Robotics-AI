/**
 * Configuration utilities for frontend code.
 *
 * BROWSER-SAFE: No process.env usage.
 * All environment values come from Docusaurus customFields or window globals.
 */

/**
 * Default API base URL for development.
 */
const DEFAULT_API_URL = 'http://localhost:8001';

/**
 * Get the API base URL.
 *
 * Priority:
 * 1. window.__BACKEND_URL__ (runtime override)
 * 2. Default to localhost for development
 *
 * NOTE: For production, set window.__BACKEND_URL__ in your deployment
 * or use a relative path if backend is on same origin.
 *
 * @returns The API base URL
 */
export function getApiBaseUrl(): string {
  // SSR-safe: Return default during server-side rendering
  if (typeof window === 'undefined') {
    return DEFAULT_API_URL;
  }

  // Check for runtime override (useful for deployment)
  const runtimeUrl = (window as any).__BACKEND_URL__;
  if (runtimeUrl && typeof runtimeUrl === 'string') {
    return runtimeUrl;
  }

  // Check if we're on a production domain (not localhost)
  const isProduction = !window.location.hostname.includes('localhost') &&
                       !window.location.hostname.includes('127.0.0.1');

  if (isProduction) {
    // In production, use relative path (same origin) or configure __BACKEND_URL__
    // For now, we'll use same origin with /api prefix
    // This assumes the backend is proxied or on the same domain
    return window.location.origin;
  }

  return DEFAULT_API_URL;
}

/**
 * Check if running in development mode.
 * Browser-safe: Uses hostname check instead of process.env.
 *
 * @returns True if in development mode
 */
export function isDevelopment(): boolean {
  if (typeof window === 'undefined') {
    return true; // Assume development during SSR
  }

  return window.location.hostname === 'localhost' ||
         window.location.hostname === '127.0.0.1' ||
         window.location.hostname.includes('.local');
}

/**
 * Check if running in production mode.
 * Browser-safe: Uses hostname check instead of process.env.
 *
 * @returns True if in production mode
 */
export function isProduction(): boolean {
  return !isDevelopment();
}
