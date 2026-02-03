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
 * Production API base URL (Vercel backend).
 */
const PRODUCTION_API_URL = 'https://backend-eta-eight-83.vercel.app';

/**
 * Get the API base URL.
 *
 * Priority:
 * 1. window.__BACKEND_URL__ (runtime override)
 * 2. Production URL if on production domain
 * 3. Default to localhost for development
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
    // In production, use the Vercel backend URL
    return PRODUCTION_API_URL;
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
