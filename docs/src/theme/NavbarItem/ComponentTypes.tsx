/**
 * Custom NavbarItem ComponentTypes
 *
 * Extends default Docusaurus navbar item types with custom types:
 * - custom-auth: Authentication navbar item (UserMenu)
 * - custom-search: Search bar navbar item
 */

import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import AuthNavbarItem from './AuthNavbarItem';
import SearchNavbarItem from './SearchNavbarItem';

export default {
  ...ComponentTypes,
  'custom-auth': AuthNavbarItem,
  'custom-search': SearchNavbarItem,
};
