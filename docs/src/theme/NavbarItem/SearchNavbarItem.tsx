/**
 * Search Navbar Item
 * Renders the search bar in the navbar.
 *
 * SSR-SAFE: Wrapped in BrowserOnly to prevent hydration issues.
 */

import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { SearchBar } from '../../components/Search';

export default function SearchNavbarItem(): JSX.Element {
  return (
    <BrowserOnly fallback={<div style={{ width: 200 }} />}>
      {() => <SearchBar />}
    </BrowserOnly>
  );
}
