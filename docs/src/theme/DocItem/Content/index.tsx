/**
 * Custom DocItem/Content wrapper with authentication guard.
 *
 * Wraps the default doc content with BookAccessGuard
 * to block unauthenticated users from viewing book content.
 */

import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { BookAccessGuard } from '@site/src/components/Auth';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  return (
    <BookAccessGuard>
      <Content {...props} />
    </BookAccessGuard>
  );
}
