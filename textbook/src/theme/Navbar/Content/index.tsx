import React, { useState } from 'react';
import { useThemeConfig } from '@docusaurus/theme-common';
import {
  splitNavbarItems,
  useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarItem from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import SearchBar from '@theme/SearchBar';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarSearch from '@theme/Navbar/Search';
import { useAuth } from '@site/src/components/AuthProvider';
import SignUpModal from '@site/src/components/SignUpModal';
import LoginModal from '@site/src/components/LoginModal';
import styles from './styles.module.css';

function useNavbarItems() {
  return useThemeConfig().navbar.items;
}

function NavbarItems({ items }: { items: any[] }) {
  return (
    <>
      {items.map((item, i) => (
        <NavbarItem {...item} key={i} />
      ))}
    </>
  );
}

function NavbarContentLayout({ left, right }: { left: React.ReactNode; right: React.ReactNode }) {
  return (
    <div className="navbar__inner">
      <div className="navbar__items">{left}</div>
      <div className="navbar__items navbar__items--right">{right}</div>
    </div>
  );
}

function AuthButtons() {
  const { isAuthenticated, user, logout, isLoading } = useAuth();
  const [showSignUp, setShowSignUp] = useState(false);
  const [showLogin, setShowLogin] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  if (isLoading) {
    return null;
  }

  if (isAuthenticated && user) {
    return (
      <div className={styles.userMenu}>
        <button
          className={styles.userButton}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label="User menu"
        >
          {user.profile?.name || user.email}
        </button>
        {showDropdown && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <div className={styles.userName}>{user.profile?.name || 'User'}</div>
              <div className={styles.userEmail}>{user.email}</div>
              {user.profile && (
                <div className={styles.userBadge}>
                  {user.profile.skill_level.charAt(0).toUpperCase() + user.profile.skill_level.slice(1)}
                </div>
              )}
            </div>
            <div className={styles.dropdownDivider} />
            <button className={styles.dropdownItem} onClick={logout}>
              Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <>
      <button className={styles.authButton} onClick={() => setShowLogin(true)}>
        Sign In
      </button>
      <button className={`${styles.authButton} ${styles.authButtonPrimary}`} onClick={() => setShowSignUp(true)}>
        Sign Up
      </button>
      <LoginModal isOpen={showLogin} onClose={() => setShowLogin(false)} />
      <SignUpModal isOpen={showSignUp} onClose={() => setShowSignUp(false)} />
    </>
  );
}

export default function NavbarContent(): JSX.Element {
  const mobileSidebar = useNavbarMobileSidebar();
  const items = useNavbarItems();
  const [leftItems, rightItems] = splitNavbarItems(items);
  const searchBarItem = items.find((item) => item.type === 'search');

  return (
    <NavbarContentLayout
      left={
        <>
          {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
          <NavbarLogo />
          <NavbarItems items={leftItems} />
        </>
      }
      right={
        <>
          <NavbarItems items={rightItems} />
          <NavbarColorModeToggle className="navbar__item" />
          {!searchBarItem && (
            <NavbarSearch>
              <SearchBar />
            </NavbarSearch>
          )}
          <AuthButtons />
        </>
      }
    />
  );
}
