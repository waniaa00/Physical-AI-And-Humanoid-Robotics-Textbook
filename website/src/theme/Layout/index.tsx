import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { ChatKit } from '@site/src/components/ChatKit';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<OriginalLayout>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatKit />
    </>
  );
}