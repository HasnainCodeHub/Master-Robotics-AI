/**
 * AuthNavbarItem - Custom navbar item for authentication.
 *
 * Renders UserMenu component which handles:
 * - Sign In button when not authenticated
 * - User dropdown when authenticated
 * - AuthModal for login/signup
 * - ProfileSettings access
 */

import React from 'react';
import { UserMenu } from '@site/src/components/Auth';

export default function AuthNavbarItem(): JSX.Element {
  return <UserMenu />;
}
